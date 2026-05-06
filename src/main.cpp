#include <Arduino.h>
#include "alert_manager.h"
#include "heart_monitor.h"
#include "audio_detect.h"
#include "fall_detect.h"
#include "power_manager.h"
#include "config.h"

// Description: 
// central coordinator of the entire system

// ── Audio/fusion event state 
//If Path A hasn't concluded by then, fusion fires anyway rather than waiting forever
#define PATH_A_RESULT_WAIT_MS   12000 // 12 second hard timeout safety net

static bool     audioEventPending = false;
static uint32_t audioEventTimeMs  = 0;
static bool     fallDuringEvent   = false;

void setup() {
    Serial.begin(115200);
    setCpuFrequencyMhz(80);   
    delay(2000);

    powerManagerInit();    // first — checks wake cause before anything else
    alertManagerInit();
    heartMonitorInit();
    fallDetectInit();
    audioDetectInit();

    Serial.println("[MAIN] All modules ready");
    Serial.println("[MAIN] Three-way fusion: audio + fall + HR active");
}

void loop() {
    // ── Always run all modules
    powerManagerUpdate();    // check for hold-to-sleep first
    heartMonitorUpdate();
    fallDetectUpdate();
    alertManagerUpdate();

    // ── Fall handling 
    if (fallDetectFallConfirmed()) {
        heartMonitorNotifyFall();    // suppress HR artefacts from impact shock
        audioDetectNotifyFall();     // suppress audio re-trigger from impact noise
        fallDetectAcknowledge();

        // Decides whether if Fall was flagged during an audio event or no 
        if (audioEventPending) {
            // Fall occurred inside an active Path A window — flag for fusion
            fallDuringEvent = true;
            Serial.println("[FUSION] Fall during audio window — flagged for fusion");
        } else {
            // Standalone fall — no audio event — send fall alert directly
            Serial.println("[MAIN] Standalone fall — sending alert");
            alertManagerSendFall(CONFIDENCE_HIGH);
        }
    }

   
    // ── Audio detection 
    //  Runs the audio detector every loop
    // If a gunshot sound is detected and we're not already processing one:
    // Arms Path A in the heart monitor
    // Sets the pending flag so we know to watch for fall and HR evidence
    // Records the timestamp
    // Clears the fall flag so we start fresh
    bool audioDetected = audioDetectUpdate();

    if (audioDetected && !audioEventPending) {
        Serial.println("[AUDIO] *** Sound event detected ***");
        Serial.print("[AUDIO] BPM at detection: ");
        Serial.println(heartMonitorGetBPM());

        heartMonitorNotifyAudioEvent();
        audioEventPending = true;
        audioEventTimeMs  = millis();
        fallDuringEvent   = false;

        Serial.println("[AUDIO] Path A armed — watching for HR + fall");
    }

    // ── Three-way fusion resolution 
    if (audioEventPending) {
        uint32_t elapsed     = millis() - audioEventTimeMs; // how long since the audio event
        int8_t   pathAResult = heartMonitorGetPathAResult(); // What Path A coluded (-2 to +2)
        bool     pathADone   = !heartMonitorPathAActive(); // Whether Path A finished on its own
        bool     timedOut    = (elapsed >= PATH_A_RESULT_WAIT_MS); // Whether the hard timeout fired

        // Countdown log every 2 seconds while waiting
        static uint32_t lastCountdown = 0;
        if (!pathADone && !timedOut && (millis() - lastCountdown >= 2000)) {
            lastCountdown = millis();
            Serial.print("[FUSION] Waiting... ");
            Serial.print(elapsed / 1000);
            Serial.print("s  BPM: ");
            Serial.print(heartMonitorGetBPM());
            Serial.print("  Path A: ");
            Serial.print(pathAResult);
            Serial.print("  Fall: ");
            Serial.println(fallDuringEvent ? "YES" : "NO");
        }

        // Act once Path A concludes OR hard timeout fires
        // Do NOT return here — rest of loop() must keep running
        if (pathADone || timedOut) {
            audioEventPending = false;

            if (timedOut && !pathADone) {
                Serial.println("[FUSION] Hard timeout — Path A did not conclude");
            }

            bool hasFall    = fallDuringEvent;
            bool strongHR   = (pathAResult == 2);
            bool flatHR     = (pathAResult == -1);
            bool noHRData   = (pathAResult == 0);

            // ── Log fusion inputs 
            Serial.println("\n[FUSION] ═══════════════════════════════════════");
            Serial.print("[FUSION] Audio=YES  Fall=");
            Serial.print(hasFall ? "YES" : "NO");
            Serial.print("  HR=");
            if      (strongHR)   Serial.println("SPIKE(+2)");
            else if (flatHR)     Serial.println("FLAT(-1/-1)");
            else                 Serial.println("NO_DATA(0)");

            // ── Confidence matrix 
            //
            // All three agree — highest confidence
            if (hasFall && strongHR) {
                Serial.println("[FUSION] GUNSHOT — CRITICAL");
                Serial.println("[FUSION] Sound + fall + strong HR spike");
                alertManagerSendGunshot(CONFIDENCE_CRITICAL);

            // Sound and fall but HR flat
            } else if (hasFall && (flatHR || noHRData)) {
                Serial.println("[FUSION] GUNSHOT — MEDIUM");
                Serial.println("[FUSION] Sound + fall — HR inconclusive");
                alertManagerSendGunshot(CONFIDENCE_MEDIUM);

            // Sound and HR spike but no fall
            } else if (!hasFall && strongHR) {
                Serial.println("[FUSION] GUNSHOT — MEDIUM");
                Serial.println("[FUSION] Sound + HR spike — no fall");
            alertManagerSendGunshot(CONFIDENCE_MEDIUM);

            // Sound but no HR or no fall
            } else {
                Serial.println("[FUSION] FALSE POSITIVE SUPPRESSED");
                Serial.print("[FUSION] Reason: ");
                if      (flatHR)   Serial.println("HR flat after sound");
                else if (noHRData) Serial.println("no HR data and no fall");
                else               Serial.println("insufficient evidence");
            }

            Serial.println("[FUSION] ═══════════════════════════════════════\n");
            fallDuringEvent = false;
        }
    }

    // ── Live BPM + baseline to BLE once per second 
    static uint32_t lastSentTime = 0;
    uint32_t now = millis();

    if (alertManagerIsConnected() && (now - lastSentTime) >= 1000) {
        lastSentTime = now;

        uint8_t bpm           = heartMonitorGetBPM();
        bool    baselineReady = heartMonitorBaselineReady();

        Serial.print("[HR] BPM: ");
        Serial.print(bpm);
        Serial.print("  Baseline: ");
        Serial.println(baselineReady ? "READY" : "BUILDING");

        String payload = "{";
        payload += "\"bpm\":"            + String(bpm)                             + ",";
        payload += "\"baseline_ready\":" + String(baselineReady ? "true" : "false");
        payload += "}";

        alertManagerSendRaw(payload.c_str());
    }

    // ── Loop rate monitor 
    static uint32_t loopCount = 0;
    static uint32_t loopTimer = 0;
    loopCount++;
    if (millis() - loopTimer >= 10000) {
        loopTimer = millis();
        Serial.print("[LOOP] calls/sec: ");
        Serial.println(loopCount);
        loopCount = 0;
    }
}