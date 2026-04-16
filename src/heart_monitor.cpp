#include "heart_monitor.h"
#include "config.h"
#include <Arduino.h>   
#include <Wire.h>
#include "MAX30105.h"        // SparkFun MAX3010x library
#include "heartRate.h"       // SparkFun beat detection algorithm

// ── MAX30102 sensor instance ─────────────────────────────────────────────────
static MAX30105 sensor;

// ── Beat detection state (SparkFun algorithm) ────────────────────────────────
static const  uint8_t BEAT_HISTORY  = 4;       // average over last 4 beats
static long   beatDeltas[BEAT_HISTORY]          = {0};
static uint8_t beatIndex                        = 0;
static long    lastBeatTime                     = 0;
static bool    sensorReady                      = false;

// ── Constants ───────────────────────────────────────────────────────────────

#define BASELINE_BUFFER_SIZE     60      // one BPM reading per second = 60s window
#define SUSTAIN_SECONDS          10      // how long drop must persist before flagging
#define PATH_A_WINDOW_MS         5000    // how long to watch HR after audio event
#define FALL_SUPPRESS_MS         2000    // ignore HR readings after fall
#define BPM_FLOOR                60      // absolute minimum before flagging
#define BASELINE_DROP_PCT        20      // % drop from baseline to flag
#define PATH_A_CHANGE_THRESHOLD  5       // BPM delta that counts as "changed" for Path A
#define UPDATE_INTERVAL_MS       1000    // sample HR once per second

// ── Internal state ──────────────────────────────────────────────────────────

// Rolling baseline buffer — stores last 60 BPM readings
static uint8_t  baselineBuffer[BASELINE_BUFFER_SIZE];
static int      baselineIndex    = 0;
static int      baselineFilled   = 0;    // counts up to BASELINE_BUFFER_SIZE then stays

// Current BPM
static uint8_t  currentBPM       = 0;

// Path B — unconsciousness tracking
static int      sustainedDropSec = 0;    // consecutive seconds below threshold
static bool     unconsciousFlag  = false;

// Suppression after fall
static bool     fallSuppressActive = false;
static uint32_t fallSuppressStart  = 0;

// Path A — post-audio HR consistency check
static bool     pathAActive        = false;
static uint32_t pathAStartMs       = 0;
static uint8_t  pathABaseBPM       = 0;  // BPM at moment of audio event
static int8_t   pathAResult        = 0;  // +1, -1, or 0

// Timing
static uint32_t lastUpdateMs       = 0;

// BPM source
// Returns 0 if reading is unreliable (motion artifact window active).
static uint8_t readBPM() {
    if (fallSuppressActive) {
        if ((millis() - fallSuppressStart) < FALL_SUPPRESS_MS) {
            return 0;
        }
        fallSuppressActive = false;
    }

    // Don't return stale init value — wait for first real beat
    if (!sensorReady) return 0;

    return currentBPM;
}

// ── Baseline helpers ─────────────────────────────────────────────────────────

static void pushBaseline(uint8_t bpm) {
    baselineBuffer[baselineIndex] = bpm;
    baselineIndex = (baselineIndex + 1) % BASELINE_BUFFER_SIZE;
    if (baselineFilled < BASELINE_BUFFER_SIZE) baselineFilled++;
}

static uint8_t calcBaseline() {
    if (baselineFilled == 0) return 0;
    uint32_t sum = 0;
    for (int i = 0; i < baselineFilled; i++) sum += baselineBuffer[i];
    return (uint8_t)(sum / baselineFilled);
}

// ── Path B — unconsciousness check ──────────────────────────────────────────

static void runPathB(uint8_t bpm) {
    if (bpm == 0) return;   // suppress during motion artifact window

    bool concernTriggered = false;

    // Absolute floor check
    if (bpm < BPM_FLOOR) {
        concernTriggered = true;
    }

    // Rolling baseline drop check
    if (heartMonitorBaselineReady()) {
        uint8_t baseline = calcBaseline();
        uint8_t dropThreshold = baseline - (baseline * BASELINE_DROP_PCT / 100);
        if (bpm < dropThreshold) {
            concernTriggered = true;
        }
    }

    // Must be sustained for SUSTAIN_SECONDS consecutive seconds
    if (concernTriggered) {
        sustainedDropSec++;
        if (sustainedDropSec >= SUSTAIN_SECONDS) {
            unconsciousFlag = true;
        }
    } else {
        sustainedDropSec = 0;   // reset streak if BPM recovers
        unconsciousFlag  = false;
    }
}

// ── Path A — post-gunshot HR consistency check ───────────────────────────────

static void runPathA(uint8_t bpm) {
    if (!pathAActive) return;
    if (bpm == 0)     return;   // suppress during motion artifact window

    uint32_t now = millis();

    if ((now - pathAStartMs) > PATH_A_WINDOW_MS) {
        // Window expired without a conclusion — treat as flat
        pathAResult = -1;
        pathAActive = false;
        return;
    }

    int delta = (int)bpm - (int)pathABaseBPM;
    if (delta < 0) delta = -delta;   // abs()

    if (delta >= PATH_A_CHANGE_THRESHOLD) {
        pathAResult = 1;    // HR changed — supports gunshot confidence
        pathAActive = false;
    }
    // If window expires without change, caught by the block above on next call
}

// ── Public API ───────────────────────────────────────────────────────────────

void heartMonitorInit() {
    baselineIndex      = 0;
    baselineFilled     = 0;
    currentBPM         = 0;   
    sustainedDropSec   = 0;
    unconsciousFlag    = false;
    fallSuppressActive = false;
    pathAActive        = false;
    pathAResult        = 0;
    lastUpdateMs       = 0;

    // I2C on ESP32-C3 default pins
    Wire.begin(5,6);

    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
        // Sensor not found — sensorReady stays false
        // heartMonitorUpdate() will skip reads gracefully
        Serial.println("[HR] MAX30102 not found — check wiring");
        return;
    }

    // ── Sensor configuration ─────────────────────────────────────────────────
    // Sample rate 100Hz, LED pulse width 411us, ADC range 4096
    // These settings balance current draw vs signal quality for BPM detection
    sensor.setup(
        60,     // LED brightness 0–255 (60 = ~1mA, enough for finger contact)
        4,      // sample average — reduces noise, library averages 4 readings
        2,      // LED mode 2 = Red + IR (needed for SparkFun beat algorithm)
        100,    // sample rate Hz
        411,    // pulse width microseconds
        4096    // ADC range
    );

    sensor.setPulseAmplitudeRed(0x0A);   // low red LED — proximity check only
    sensor.setPulseAmplitudeGreen(0);    // green off — not needed for HR

    sensorReady = true;
    Serial.println("[HR] MAX30102 ready");

}

void heartMonitorUpdate() {
    uint32_t now = millis();

    // ── Sample collection — runs every loop(), not rate limited ──────────────
    // checkForBeat() needs 100Hz sample rate to detect peaks reliably
    if (sensorReady) {
        long irValue = sensor.getIR();

        if (irValue > 50000) {
            if (checkForBeat(irValue)) {
                long delta = now - lastBeatTime;
                lastBeatTime = now;

                if (delta > 300 && delta < 2000) {
                    beatDeltas[beatIndex % BEAT_HISTORY] = delta;
                    beatIndex++;

                    if (beatIndex >= BEAT_HISTORY) {
                        long avgDelta = 0;
                        for (uint8_t i = 0; i < BEAT_HISTORY; i++) avgDelta += beatDeltas[i];
                        avgDelta /= BEAT_HISTORY;
                        uint8_t bpm = (uint8_t)(60000 / avgDelta);

                        if (bpm >= 30 && bpm <= 220) {
                            currentBPM = bpm;   // update current BPM immediately on valid beat
                        }
                    }
                }
            }
        }
    }

    // ── 1 second tick — baseline + path logic ────────────────────────────────
    if ((now - lastUpdateMs) < UPDATE_INTERVAL_MS) return;
    lastUpdateMs = now;

    uint8_t bpm = readBPM();   // returns currentBPM or 0 if suppressed

    if (bpm > 0) {
        pushBaseline(bpm);
    }

    runPathB(bpm);
    runPathA(bpm);
}

uint8_t heartMonitorGetBPM() {
    return currentBPM;
}

bool heartMonitorBaselineReady() {
    return baselineFilled >= BASELINE_BUFFER_SIZE;
}

bool heartMonitorUnconsciousFlag() {
    return unconsciousFlag;
}

void heartMonitorNotifyFall() {
    fallSuppressActive = true;
    fallSuppressStart  = millis();
}

void heartMonitorNotifyAudioEvent() {
    pathAActive    = true;
    pathAStartMs   = millis();
    pathABaseBPM   = currentBPM;
    pathAResult    = 0;
}

int8_t heartMonitorGetPathAResult() {
    return pathAResult;
}

