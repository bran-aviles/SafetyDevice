#include "heart_monitor.h"
#include "config.h"
#include <Arduino.h>   
#include <Wire.h>
#include "MAX30105.h"        // SparkFun MAX3010x library
#include "heartRate.h"       // SparkFun beat detection algorithm contains checkforBeat()


//Description:
// MAX30102 sensor shines infrared light into the skin 400 times per second
// each heartbeat creates a distinct pulse in that IR signal.
//
// The code measures the time between those pulses and divides 60,000 by that gap to get BPM
// Before trusting any readings it waits for a stable finger, discards the first 6 warmup beats
// filters out motion artifacts using accelerometer data
// runs a 10-beat rolling average to keep the display smooth
//
// Smoothing filter only applies to the display
//
//  Path A — the adrenaline detector — sees every raw beat first, unfiltered
//  because a genuine stress response is exactly the large sudden jump the filter would have thrown away.
//
// Path A activates the moment a gunshot sound is detected
//  It records the person's baseline HR and watches for up to 30 seconds
//  scoring the biological response from -2 to +2.
//  That score feeds into the fusion system alongside the audio evidence to make a final confidence decision

// ── Sensor instance 
static MAX30105 sensor;  // Actual MAX30105 sensor object
static bool     sensorReady = false; // Safety Flag

// ── Heart rate state 
static uint8_t  currentBPM       = 0;

// ── BPM smoothing 
// Keeps the last 10 BPM readings in an array and averages them
// Smoothes out the beat to beat BPM (raw beat to beat jumps around a lot)
#define BPM_SMOOTH_SIZE      10
static uint8_t  bpmHistory[BPM_SMOOTH_SIZE] = {0}; // array storing last 10 readings
static uint8_t  bpmHistoryIndex  = 0; // which slot to write the next reading into
static uint8_t  bpmHistoryFilled = 0; // how many slots have real data so far

// ── Beat detection state 
#define BEAT_HISTORY               6  // throw away the first 6 beats while the algorithm "warms up" and stabilizes
static long     lastBeatTime     = 0; // timestamp of the last detected beat, used to calculate time between beats
static uint8_t  beatIndex        = 0; // counts how many warmup beats have passed
static bool     warmupDone       = false;  // flag for "are we past warmup yet"

// ── Finger detection 
static bool     fingerPresent        = false; 
static bool     fingerStable         = false;
static uint8_t  fingerStableCount    = 0;
#define FINGER_STABLE_THRESHOLD      10 // needs 10 consecutive good readings before declared stable

// ── Baseline Buffer
// Stores the last 60 seconds of BPM readings (one per second)
// Gives a reliable "normal" heart rate for the person
#define BASELINE_BUFFER_SIZE  60
static uint8_t  baselineBuffer[BASELINE_BUFFER_SIZE];
static int      baselineIndex    = 0;
static int      baselineFilled   = 0;

// ── Fall suppression 
// If a fall is detected, ignore heart rate readings for 2 seconds
static bool     fallSuppressActive = false;
static uint32_t fallSuppressStart  = 0;
#define FALL_SUPPRESS_MS             2000

// ── Path A — post-gunshot HR consistency
// When a gunshot sound is detected, it watches to see if the person's heart rate spikes
static bool     pathAActive      = false;
static uint32_t pathAStartMs     = 0;
static uint8_t  pathABaseBPM     = 0;
static int8_t   pathAResult      = 0;

// ── Timing 
static uint32_t lastUpdateMs          = 0;
#define UPDATE_INTERVAL_MS            1000

// ── Outlier rejection state 
static uint8_t  consecutiveRejections = 0;

// ── Motion-gated beat suppression 
// If the person's wrist moves, it creates fake "beats" in the sensor
static float    lastMotionG      = 1.0f;
static float    peakMotionG      = 1.0f; // highest motion reading in the last 150ms
static uint32_t peakMotionMs     = 0; 
#define MOTION_BEAT_SUPPRESS_G   1.08f // anything above 1.08g means too much motion, ignore bats
#define MOTION_PEAK_HOLD_MS      150 // hold the peak for 150ms because the motion spike and the fake beat don't arrive at exactly the same time

// ── Adaptive outlier window
// Calculates standard deviation of recent BPM readings 
// Used to automatically tighten or loosen the outlier rejection window. 
// Stable signal = tighter window, variable signal = wider window
// If less than 3 readings available, returns a safe default of 6
static float computeStdDev() {
    if (bpmHistoryFilled < 3) return 6.0f;   // not enough data — use safe default
    float mean = 0;
    for (byte i = 0; i < bpmHistoryFilled; i++) mean += bpmHistory[i];
    mean /= bpmHistoryFilled;
    float variance = 0;
    for (byte i = 0; i < bpmHistoryFilled; i++) {
        float d = (float)bpmHistory[i] - mean;
        variance += d * d;
    }
    return sqrtf(variance / bpmHistoryFilled);
}

// ── Baseline helpers 
// Adds a new BPM reading to the 60-second rolling baseline
// The % BASELINE_BUFFER_SIZE makes the index wrap around 
// when it hits slot 60 it goes back to slot 0, overwriting the oldest reading.
// Like a circular conveyor belt.
static void pushBaseline(uint8_t bpm) {
    baselineBuffer[baselineIndex] = bpm;
    baselineIndex = (baselineIndex + 1) % BASELINE_BUFFER_SIZE;
    if (baselineFilled < BASELINE_BUFFER_SIZE) baselineFilled++;
}

// Returns the average of however many baseline samples have been collected.
// Used by Path A to get a stable pre-event HR reference rather than relying
// on the instantaneous currentBPM at the moment the audio event fires.
static uint8_t getBaselineAverage() {
    if (baselineFilled == 0) return 0;
    uint32_t sum = 0;
    for (int i = 0; i < baselineFilled; i++) sum += baselineBuffer[i];
    return (uint8_t)(sum / baselineFilled);
}

// ── Fast BPM tracker — for Path A only
// The main smoothing buffer (BPM_SMOOTH_SIZE=10) is deliberately slow to
// keep the displayed reading stable. Path A needs to detect a sudden HR
// spike within seconds of an audio event, so we maintain a separate 3-beat
// fast average that reacts quickly without polluting the stable reading.
#define FAST_BPM_SIZE        3
static uint8_t  fastBpmHistory[FAST_BPM_SIZE] = {0};
static uint8_t  fastBpmIndex    = 0;
static uint8_t  fastBpmFilled   = 0;
static uint8_t  fastBPM         = 0;

static void pushFastBPM(uint8_t bpm) {
    fastBpmHistory[fastBpmIndex % FAST_BPM_SIZE] = bpm;
    fastBpmIndex++;
    if (fastBpmFilled < FAST_BPM_SIZE) fastBpmFilled++;
    uint16_t sum = 0;
    for (byte i = 0; i < fastBpmFilled; i++) sum += fastBpmHistory[i];
    fastBPM = (uint8_t)(sum / fastBpmFilled);
}

static void resetFastBPM() {
    fastBpmIndex  = 0;
    fastBpmFilled = 0;
    fastBPM       = 0;
    for (byte i = 0; i < FAST_BPM_SIZE; i++) fastBpmHistory[i] = 0;
}

// ── Path A — post-gunshot HR check
// Called on every accepted beat (not just the 1s tick) so it reacts as
// fast as the sensor allows rather than waiting up to 1s between checks.
//
// Result values:
//   +2  = strong spike ≥15 BPM  — Critical gunshot confidence
//   -1  = Sound + fall, but HR flat
//   -1  = Sound + HR spike, but no fall
// 
//
// IMPORTANT FOR FUSION: pathAResult = 0 while pathAActive = true means
// "still watching, not enough time has passed". Fusion must check
// heartMonitorPathAActive() before treating 0 as inconclusive — if Path A
// is no longer active and result is 0, something went wrong (finger lifted).
//
// Early flat conclusion at 10s:
// If 10 seconds have passed with rise < 8 BPM, we conclude -1 immediately
// rather than waiting the full 30s. A real adrenaline response is detectable
// within 5–15s — if nothing has happened by 10s, HR is flat. This prevents
// fusion from seeing result=0 and treating it as no evidence when it should
// be treated as negative evidence.
//
// Full window (30s) is only needed to catch delayed moderate rises (8–14 BPM)
// which take longer to build and confirm.

static void runPathA(uint8_t bpm) {
    if (!pathAActive) return;
    if (bpm == 0)     return;
    if (fastBPM == 0) return;

    uint32_t now     = millis();
    uint32_t elapsed = now - pathAStartMs;

    // Require at least 3 fast samples so a single noisy beat can't decide
    if (fastBpmFilled < 3) return;

    // Calculation on how many BPM has the heart rate risen since the audio event fired
    int rise = (int)fastBPM - (int)pathABaseBPM;

    Serial.print("[HR] Path A check — base: ");
    Serial.print(pathABaseBPM);
    Serial.print("  fast: ");
    Serial.print(fastBPM);
    Serial.print("  rise: ");
    Serial.print(rise);
    Serial.print("  elapsed: ");
    Serial.print(elapsed / 1000);
    Serial.println("s");

    // ── Strong spike — conclude immediately (+2)
    if (rise >= 15) {
        pathAResult = 2;
        pathAActive = false;
        Serial.print("[HR] Path A — strong HR spike +");
        Serial.print(rise);
        Serial.println(" BPM — CRITICAL high gunshot confidence");
        return;
    }

    // ── Early flat conclusion at 10s (-1)
    // If HR is below +15 BPM after 10 seconds, HR is flat.
    if (elapsed > 10000) {
        pathAResult = -1;
        pathAActive = false;
        Serial.print("[HR] Path A — no meanigful spike, rise only ");
        Serial.print(rise);
        Serial.println(" BPM - MEDIUM gunshot confidence");
        return;
    }

    // ── Full window expiry (30s) — catch delayed moderate rises 
    if (elapsed > PATH_A_WINDOW_MS) {
        pathAResult = -1;
        pathAActive = false;
        Serial.print("[HR] Path A — window expired, no spike");
    }
}

// ── Reset heart rate tracking 

static void resetBeatTracking() {
    beatIndex              = 0;
    warmupDone             = false;
    lastBeatTime           = 0;
    bpmHistoryIndex        = 0;
    bpmHistoryFilled       = 0;
    fingerStableCount      = 0;
    fingerStable           = false;
    fingerPresent          = false;
    consecutiveRejections  = 0;
    for (byte i = 0; i < BPM_SMOOTH_SIZE; i++) bpmHistory[i] = 0;
    currentBPM = 0;
    resetFastBPM();
}

// ── Public API

void heartMonitorInit() {
    baselineIndex      = 0;
    baselineFilled     = 0;
    fallSuppressActive = false;
    pathAActive        = false;
    pathAResult        = 0;
    lastUpdateMs       = 0;
    sensorReady        = false;
    lastMotionG        = 1.0f;
    peakMotionG        = 1.0f;
    peakMotionMs       = 0;

    resetBeatTracking();

    Wire.begin(5, 6);
    Wire.setClock(400000);

    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("[HR] MAX30102 not found — check wiring");
        return;
    }

    // ── Medium-power setup 
    // sampleAverage = 4  → hardware averages 4 raw samples per FIFO entry.
    //   Cuts noise before it ever reaches beat detection. At 400Hz input
    //   this gives an effective ~100Hz beat-detection sample rate, which is
    //   plenty for HR and removes high-frequency motion artefacts.
    //
    // ledBrightness = (~5 mA).
    //   Enough signal for fingertip placement; lower brightness reduces
    //   ambient-light sensitivity and heat from the LED package

    sensor.setup(
        0x24,   // ledBrightness — ~5.8mA (bright enough for reliable readings)
        4,      // sampleAverage — hardware avg 4 samples
        2,      // ledMode 2 — Red + IR only
        400,    // sampleRate
        411,    // pulseWidth
        4096    // adcRange
    );
    sensor.setPulseAmplitudeRed(0x0A);
    sensor.setPulseAmplitudeGreen(0);   // Green off — not used

    sensorReady = true;
    Serial.println("[HR] MAX30102 ready — medium power, 4x HW averaging");
}

void heartMonitorUpdate() {
    uint32_t now = millis();

    if (sensorReady) {

        // ── Fall suppression check 
        // If suppressed, keep draining the sensor buffer but don't process any readings
        bool suppressed = false;
        if (fallSuppressActive) {
            if ((now - fallSuppressStart) < FALL_SUPPRESS_MS) {
                suppressed = true;
            } else {
                fallSuppressActive = false;
            }
        }

        // ── FIFO drain 
        // Sensor stores readings in a FIFO buffer (like a queue)
        sensor.check(); // checks new data
        while (sensor.available()) {
            long irValue = sensor.getFIFOIR(); // new infrared light reading
            sensor.nextSample();

            if (suppressed) continue;

            // ── Finger detection 
            // IR below 5000 = no finger
            // Reset everyting and skip
            if (irValue < 5000) {
                if (fingerPresent) {
                    resetBeatTracking();
                    Serial.println("[HR] Finger removed — resetting");
                }
                continue;
            }

            fingerPresent = true;

            // ── Stabilisation gate ─
            // Wait for 10 consecutive samples above 60000 before trusting signal.
            if (!fingerStable) {
                if (irValue > 60000) {
                    fingerStableCount++;
                    if (fingerStableCount >= FINGER_STABLE_THRESHOLD) {
                        fingerStable = true;
                        Serial.println("[HR] Finger stable — beat detection starting");
                    }
                } else {
                    fingerStableCount = 0;
                }
                continue;
            }

            // ── Sample rate cap: process at most 100 samples/sec 
            // With 4x HW averaging the FIFO delivers ~100 entries/sec, so
            // this guard rarely fires - but kept as a safety net.
            static uint32_t lastSampleMs = 0;
            uint32_t nowMs = millis();
            if ((nowMs - lastSampleMs) < 10) continue;   // was 2 ms (500Hz) — now 10 ms (100Hz)
            lastSampleMs = nowMs;

            // ── Debug: IR value every 2s
            static uint32_t lastBeatDebug = 0;
            if (millis() - lastBeatDebug > 2000) {
                lastBeatDebug = millis();
                Serial.print("[HR] In beat block — IR: ");
                Serial.println(irValue);
            }

            // checkForBeat() is SparkFun's algorithm
            // returns true when it detects a heartbeat in the IR signal
            // delta is the time in milliseconds since the last beat
            if (checkForBeat(irValue)) {
                long now_ms  = millis();
                long delta   = now_ms - lastBeatTime;

                // Guard against stale/zero lastBeatTime on first beat
                if (lastBeatTime == 0) {
                    lastBeatTime = now_ms;
                    continue;
                }

                // ── Motion gate (peak-hold)
                // Use the 150ms peak rather than the instantaneous reading —
                // see comment on peakMotionG above for why this matters.
                if (peakMotionG > MOTION_BEAT_SUPPRESS_G) {
                    Serial.print("[HR] Beat suppressed — motion peak: ");
                    Serial.print(peakMotionG, 3);
                    Serial.println("g");
                    continue;
                }

                // ── Minimum beat interval guard 
                // At 150 BPM max the shortest real inter-beat interval is
                // 60000/150 = 400ms. Guard at 400ms exactly — double-fires
                // from checkForBeat() on the stronger IR signal (138,000+)
                // were arriving at 50–342ms, all safely below this floor.
                // Real beats at 150 BPM arrive at exactly 400ms so we use
                // strict < rather than <= to let the boundary beat through.
                if (delta < 400) {
                    Serial.print("[HR] Beat ignored — too soon: ");
                    Serial.print(delta);
                    Serial.println(" ms");
                    continue;
                }

                //Converts time between beats into BPM
                //60,000 ms is chosen because these are the amount of ms in a minute
                // BPM = 60,000ms /  time between beats (ms)
                float bpmFloat = 60000.0f / (float)delta;   // delta already in ms

                // ── HR window: 40–150 BPM 
                // Resting: 55–85 BPM. Light activity: 85–110. Moderate
                // exercise: 110–140. Hard effort: 140–150+.
                // Lower floor at 40 covers athletes with low resting HR.
                // Upper ceiling at 150 covers hard physical effort while
                // still rejecting noise above that range.
                if (bpmFloat >= 40.0f && bpmFloat <= 150.0f) {
                    uint8_t bpm = (uint8_t)(bpmFloat + 0.5f);   // round, don't truncate

                    // ── Warmup gate 
                    // Discard first BEAT_HISTORY beats while the algo locks on.
                    if (!warmupDone) {
                        beatIndex++;
                        lastBeatTime = now_ms;
                        consecutiveRejections = 0;
                        Serial.print("[HR] Warmup beat ");
                        Serial.print(beatIndex);
                        Serial.print("/");
                        Serial.print(BEAT_HISTORY);
                        Serial.print(" — raw: ");
                        Serial.println(bpm);
                        if (beatIndex >= BEAT_HISTORY) {
                            warmupDone = true;
                            Serial.println("[HR] Warmup complete — accepting beats");
                        }
                        continue;
                    }

                    // ── Always feed Path A before outlier gate 
                    // Path A needs to see extreme spikes (15+ BPM jumps) that are caused
                    // by genuine adrenaline response — e.g. person has been shot.
                    // The outlier gate exists for display stability only and must not
                    // suppress evidence that Path A is specifically looking for.
                    if (pathAActive) {
                        pushFastBPM(bpm);
                        runPathA(bpm);
                    }

                    // ── Outlier rejection ──  display smoother only
                    // If the new beat is more than 15 BPM away from the current reading, throw it away
                    // After 3 consecutive rejections, reset the counter and wait for a real beat
                    if (currentBPM > 0) {
                        int diff = (int)bpm - (int)currentBPM;
                        if (diff < -15 || diff > 15) {
                            consecutiveRejections++;
                            Serial.print("[HR] Beat rejected — outlier: ");
                            Serial.print(bpm);
                            Serial.print("  current: ");
                            Serial.print(currentBPM);
                            Serial.print("  diff: ");
                            Serial.print(diff);
                            Serial.print("  (");
                            Serial.print(consecutiveRejections);
                            Serial.println(" consecutive)");

                            // After 3 consecutive rejects the algo has truly
                            // lost lock — clear the counter but do NOT touch
                            // lastBeatTime so the next real beat still gets a
                            // clean delta from the last accepted beat.
                            if (consecutiveRejections >= 3) {
                                consecutiveRejections = 0;
                                Serial.println("[HR] Rejection counter cleared — waiting for real beat");
                            }
                            continue;
                        }
                    }

                    // ── Accepted beat
                    consecutiveRejections = 0;
                    lastBeatTime = now_ms;

                    // Slow rolling average — stable display reading
                    // Adds the beat to the rolling average and recalculates current BPM
                    bpmHistory[bpmHistoryIndex % BPM_SMOOTH_SIZE] = bpm;
                    bpmHistoryIndex++;
                    if (bpmHistoryFilled < BPM_SMOOTH_SIZE) bpmHistoryFilled++;
                    uint32_t sum = 0;
                    for (byte i = 0; i < bpmHistoryFilled; i++) sum += bpmHistory[i];
                    currentBPM = (uint8_t)(sum / bpmHistoryFilled);

                    // / Only push to fast tracker if Path A isn't active
                    if (!pathAActive) pushFastBPM(bpm);

                    Serial.print("[HR] Beat — raw: ");
                    Serial.print(bpm);
                    Serial.print("  smoothed: ");
                    Serial.print(currentBPM);
                    Serial.print("  fast: ");
                    Serial.println(fastBPM);

                    // Run Path A on every accepted beat so it reacts within
                    // seconds rather than waiting for the 1s tick
                    if (pathAActive) runPathA(bpm);
                }
                // Beats outside 40–150 still update lastBeatTime so the
                // next inter-beat interval is calculated correctly.
                else {
                    lastBeatTime = now_ms;
                }
            }
        }
    }

    // ── 1 second tick 
    if ((now - lastUpdateMs) < UPDATE_INTERVAL_MS) return;
    lastUpdateMs = now;

    if (currentBPM > 0) pushBaseline(currentBPM);

    // Path A is now checked per accepted beat above for faster response.
    // The 1s tick handles the window-expiry case when no beats are arriving
    // (e.g. finger lifted mid-window).
    if (pathAActive) runPathA(currentBPM);
}

// ── Getters 

uint8_t heartMonitorGetBPM() {
    return currentBPM;
}

bool heartMonitorBaselineReady() {
    return baselineFilled >= BASELINE_BUFFER_SIZE;
}

int8_t heartMonitorGetPathAResult() {
    return pathAResult;
}

// Returns true while Path A is still watching — fusion must check this
// before interpreting pathAResult = 0 as "inconclusive". If this returns
// false and result is 0, the finger was lifted mid-window.
bool heartMonitorPathAActive() {
    return pathAActive;
}

// ── External notifications
// Called by fall detector, suppresses HR readings for 2 seconds
void heartMonitorNotifyFall() {
    fallSuppressActive = true;
    fallSuppressStart  = millis();
    Serial.println("[HR] Fall notified — suppressing readings 2 seconds");
}
// Called every loop by the IMU, feeds urrent acceleration so the motion gate can suppress fake beats during wrist movement
void heartMonitorSetMotion(float motionG) {
    lastMotionG = motionG;
    uint32_t now = millis();
    // Update peak: raise immediately on new high, decay after hold window
    if (motionG > peakMotionG || (now - peakMotionMs) > MOTION_PEAK_HOLD_MS) {
        peakMotionG  = motionG;
        peakMotionMs = now;
    }
}

// Called when a gunshot sound is detected — arms Path A
// Records the baseline BPM as the reference point and clears the fast buffer so only post-event beats count
void heartMonitorNotifyAudioEvent() {
    pathAActive  = true;
    pathAStartMs = millis();
    pathAResult  = 0;

    // Use the 60s rolling baseline average as the reference point if we
    // have at least 10 seconds of history (10 samples at 1/sec).
    // This is more reliable than instantaneous currentBPM which could be
    // temporarily elevated (person just walked over, just sat down, etc).
    // Falls back to currentBPM for the first 10s of operation.
    if (baselineFilled >= 10) {
        pathABaseBPM = getBaselineAverage();
        Serial.print("[HR] Path A armed — baseline BPM: ");
    } else {
        pathABaseBPM = currentBPM;
        Serial.print("[HR] Path A armed — base BPM (no baseline yet): ");
    }
    Serial.println(pathABaseBPM);

    // Clear the fast buffer so Path A only measures beats that arrive
    // *after* the audio event — pre-event readings would dilute a real spike
    resetFastBPM();
}