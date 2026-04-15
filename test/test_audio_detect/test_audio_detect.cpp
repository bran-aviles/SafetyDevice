#include <Arduino.h>
#include <unity.h>
#include "audio_detect.h"

// ── Simulated test buffers (all values pre-shift: intended_value × 64) ──────
//
// runAudioDetect() applies >> 6 internally before comparing thresholds.
// Buffer values here are raw (pre-shift) so after >> 6 they match config.h.
//
//   262000 × 64 = 16768000   → post-shift 262000  (above CLIP_THRESHOLD 249037)
//   260000 × 64 = 16640000   → post-shift 260000  (above CLIP_THRESHOLD)
//     1000 × 64 =    64000   → post-shift   1000  (ambient)
//    80000 × 64 =  5120000   → post-shift  80000  (below DECAY_THRESHOLD 100000)
//    30000 × 64 =  1920000   → post-shift  30000
//     8000 × 64 =   512000   → post-shift   8000

// Test 1 — clean gunshot — should DETECT
int32_t testGunshot[] = {
    64000,  76800,  70400, 57600, 51200,  // ambient (1000–1200 post-shift)
    16640000, 16768000,                   // fast spike — diff > ONSET_DIFF post-shift
    16768000, 16768000, 16768000,         // flatline — 5 samples
    16768000, 16768000,                   // flatline continues — 7 total
    5120000, 1920000, 512000, 64000       // decay — drops below DECAY_THRESHOLD
};

// Test 2 — gradual rise — should NOT detect
// No single step exceeds ONSET_DIFF (200000) post-shift
int32_t testGradual[] = {
    64000,   960000,  2560000,  5120000,
    8320000, 12160000, 15360000,
    16768000, 16768000, 16768000,
    16768000, 16768000, 16768000,
    3200000, 640000, 64000
};

// Test 3 — sustained loud noise — should NOT detect
// Filled in setUp(): flatline > MAX_FLATLINE (80) triggers reset
int32_t testSustained[100];

// Test 4 — ambient only — should NOT detect
int32_t testAmbient[] = {
    32000,  51200,  76800, 44800,
    57600,  70400,  38400, 51200,
    64000,  48000,  54400, 60800
};

// ── Unity lifecycle ─────────────────────────────────────────────────────────

void setUp() {
    testSustained[0] = 64000;       // ambient (1000 post-shift)
    testSustained[1] = 16640000;    // spike — onset triggers (260000 post-shift)
    for (int i = 2; i < 100; i++) {
        testSustained[i] = 16768000; // 98 clipped samples — resets when > MAX_FLATLINE (80)
    }
}

void tearDown() {}

// ── Test cases ──────────────────────────────────────────────────────────────

void test_gunshot_detected() {
    TEST_ASSERT_TRUE(runAudioDetect(testGunshot, sizeof(testGunshot) / sizeof(int32_t)));
}

void test_gradual_rise_not_detected() {
    TEST_ASSERT_FALSE(runAudioDetect(testGradual, sizeof(testGradual) / sizeof(int32_t)));
}

void test_sustained_noise_not_detected() {
    TEST_ASSERT_FALSE(runAudioDetect(testSustained, 100));
}

void test_ambient_not_detected() {
    TEST_ASSERT_FALSE(runAudioDetect(testAmbient, sizeof(testAmbient) / sizeof(int32_t)));
}

// ── Entry point (Arduino-style for ESP32-C3) ────────────────────────────────

void setup() {
    delay(2000);        // allow USB-CDC to connect before output starts
    UNITY_BEGIN();
    RUN_TEST(test_gunshot_detected);
    RUN_TEST(test_gradual_rise_not_detected);
    RUN_TEST(test_sustained_noise_not_detected);
    RUN_TEST(test_ambient_not_detected);
    UNITY_END();
}

void loop() {}