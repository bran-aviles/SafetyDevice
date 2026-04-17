#pragma once

// ── Audio ──────────────────────────────────────────────────────────────────
#define SAMPLE_RATE         32000   // minimum supported by SPH0645
#define SAMPLE_BUFFER_SIZE  256     // 8ms window at 32kHz
#define ADC_MAX             262144  // 18-bit maximum
#define CLIP_THRESHOLD      249037  // 95% of ADC_MAX
#define ONSET_DIFF          200000  // minimum jump for fast onset
#define MIN_FLATLINE        5       // 0.15ms at 32kHz
#define MAX_FLATLINE        80      // 2.5ms at 32kHz
#define DECAY_THRESHOLD     100000  // signal must drop below this after flatline
#define DECAY_WINDOW        20      // 0.6ms at 32kHz


// ── Heart Monitor ──────────────────────────────────────────────────────────
#define HR_BASELINE_WINDOW_SEC    60    // rolling baseline window
#define HR_UNRELIABLE_AFTER_FALL  2000  // ms to ignore HR after fall
#define HR_DROP_THRESHOLD_PCT     20    // % drop from baseline → unconscious concern

// ── Fall Detect ────────────────────────────────────────────────────────────
#define FALL_FREEFALL_THRESHOLD   0.3f  // g
#define FALL_IMPACT_THRESHOLD     3.0f  // g

// ── Fusion / Alert ─────────────────────────────────────────────────────────
#define COOLDOWN_MS               10000
#define CO_OCCURRENCE_WINDOW_MS   5000
#define CONFIDENCE_HIGH           75
#define CONFIDENCE_MED            40

// ── BLE ────────────────────────────────────────────────────────────────────
#define BLE_DEVICE_NAME     "SafetyDevice"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID           "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define COOLDOWN_MS              10000
#define CO_OCCURRENCE_WINDOW_MS  5000

// ── RGB LED 
#define LED_PIN          8    // WS2812 data pin on ESP32-C3 DevKit
#define LED_COUNT        1    // one onboard LED
#define LED_BRIGHTNESS   50   // 0–255, keep low to avoid washing out

// ── LSM6DSOX / Fall Detect ─────────────────────────────────────────────────
#define LSM_INT1_PIN          9      // GPIO9 — LSM6DSOX INT1 interrupt pin
#define LSM_I2C_ADDR          0x6A   // SDO tied to GND = 0x6A 
#define FALL_CONFIRM_MS       100   // ms window to confirm fall after interrupt
#define FALL_COOLDOWN_MS      5000  // ms before another fall can be detected