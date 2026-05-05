#pragma once

// ── Audio 
#define I2S_SAMPLE_RATE     32000   // minimum supported by SPH0645
// A gunshot's acoustic signature duration can last around 3 - 5 milliseconds 
// With 8ms, it fits the entire gunshot shape with room to spare
#define I2S_BUFFER_SIZE     256     // 8ms window at 32kHz (256/32,000) = 0.008 seconds = 8ms
#define ADC_MAX             262144  // 18-bit maximum
#define CLIP_THRESHOLD      249037  // 95% of ADC_MAX
#define ONSET_DIFF          247000  // minimum jump for fast onset
#define MIN_FLATLINE        8       // 0.15ms at 32kHz
#define MAX_FLATLINE        80      // 2.5ms at 32kHz
#define DECAY_THRESHOLD     150000  // sig        nal must drop below this after flatline
#define DECAY_WINDOW        8      // 0.25ms at 32kHz
#define I2S_BCLK        3     // Bit clock
#define I2S_LRCL        2     // Left/right clock (word select)
#define I2S_DOUT        4     // Data out from mic
#define I2S_PORT        I2S_NUM_0
#define AUDIO_FALL_SUPPRESS_MS          3000   // suppress audio 3 seconds after fall


// ── Heart Monitor 
#define HR_BASELINE_WINDOW_SEC    60
#define FALL_SUPPRESS_MS          2000
#define PATH_A_WINDOW_MS          30000   // 10s — captures full startle response

// ── Fall Detect
#define FALL_FREEFALL_THRESHOLD   0.3f  // g
#define FALL_IMPACT_THRESHOLD     3.0f  // g
#define FREE_FALL_MIN_MS    80    // minimum ms below 0.4g to confirm free-fall

// Confidence levels — must be strings, passed as JSON values in BLE payload
#define CONFIDENCE_MEDIUM         "MEDIUM"
#define CONFIDENCE_CRITICAL       "CRITICAL"

// ── BLE 
#define BLE_DEVICE_NAME     "SafetyDevice"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID           "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define COOLDOWN_MS              10000  // 10s between same alert type


// ── RGB LED 
#define LED_PIN          8    // WS2812 data pin on ESP32-C3 DevKit
#define LED_COUNT        1    // one onboard LED
#define LED_BRIGHTNESS   50   // 0–255, keep low to avoid washing out

// ── LSM6DSOX / Fall Detect
#define LSM_INT1_PIN          9      // GPIO9 — LSM6DSOX INT1 interrupt pin
#define LSM_I2C_ADDR          0x6A   // SDO tied to GND = 0x6A 
#define FALL_CONFIRM_MS       100   // ms window to confirm fall after interrupt
#define FALL_COOLDOWN_MS      5000  // ms before another fall can be detected