#include <Arduino.h>
#include "alert_manager.h"
#include "heart_monitor.h"

void setup() {
    Serial.begin(115200);
    delay(2000);

    alertManagerInit();
    heartMonitorInit();
}

void loop() {
    // ── Always run heart monitor — sensor reads happen at 100Hz ──────────
    heartMonitorUpdate();
    alertManagerUpdate();

     // ── Print sensor prompt once on first BLE connection ──────────────────
    static bool promptShown = false;
    if (alertManagerIsConnected() && !promptShown) {
        promptShown = true;
        Serial.println("[MAIN] Place finger on MAX30102 sensor");
    }

    // ── Send BLE payload once per second when connected ───────────────────
    static uint32_t lastSend = 0;
    if (alertManagerIsConnected() && millis() - lastSend >= 1000) {
        lastSend = millis();

        uint8_t bpm           = heartMonitorGetBPM();
        bool    baselineReady = heartMonitorBaselineReady();
        bool    unconscious   = heartMonitorUnconsciousFlag();

        // Always print to serial regardless of BLE connection
        Serial.print("[HR] BPM: ");
        Serial.print(bpm);
        Serial.print("  Baseline: ");
        Serial.print(baselineReady ? "READY" : "BUILDING");
        Serial.print("  Unconscious: ");
        Serial.println(unconscious ? "YES" : "NO");

        // Send over BLE only if connected
        if (alertManagerIsConnected()) {
            String payload = "{";
            payload += "\"type\":\"HEART\",";
            payload += "\"bpm\":"            + String(bpm)                              + ",";
            payload += "\"baseline_ready\":" + String(baselineReady ? "true" : "false") + ",";
            payload += "\"unconscious\":"    + String(unconscious   ? "true" : "false");
            payload += "}";

            alertManagerSendRaw(payload.c_str());
        }
    }

    // ── Fire unconscious alert when flag sets ─────────────────────────────
    static bool lastUnconsciousState = false;
    bool unconsciousNow = heartMonitorUnconsciousFlag();

    if (unconsciousNow && !lastUnconsciousState) {
        Serial.println("[MAIN] Unconscious flag SET — sending alert");
        alertManagerSendUnconscious(CONFIDENCE_HIGH);
    }
    if (!unconsciousNow && lastUnconsciousState) {
        Serial.println("[MAIN] Unconscious flag CLEARED");
    }
    lastUnconsciousState = unconsciousNow;
}