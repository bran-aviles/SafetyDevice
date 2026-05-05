#include "fall_detect.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <math.h>

// Description:
// This file reads from an accelerometer chip and detects falls using a two stage pattern:
// Weightlessness - the person is in the air, total g-force drops near zero for at least 80ms
// Impact - the person hits the ground, total g-force spikes above 2.5g within 600ms


// ── Sensor instance
static Adafruit_LSM6DSOX imu; // Actual IMU sensor object
static bool sensorReady = false;

// ── Fall detection state 
static bool     fallConfirmed   = false;
static uint32_t lastFallMs      = 0;
static bool     freeFallSeen    = false;
static uint32_t freeFallTimeMs  = 0;
static uint32_t freeFallEntryMs = 0;

// ── Forward declaration 
static void writeReg(uint8_t reg, uint8_t val);

// ── Raw magnitude 
// Reads the accelerometer 
// returns a signal number representing total force in g's
//   At rest:   ~1.0g
//   Free-fall: ~0.0g  (weightless)
//   Impact:    3.0–8.0g+
static float getRawMagnitude() {
    sensors_event_t accel, gyro, temp;
    // reads all sensor data, acceleration, gyroscope, and temperature
    imu.getEvent(&accel, &gyro, &temp); 
    // Gives acceleration in m/s^2, divided by 9.81 converts to g-force (1g = normal gravity)
    float x = accel.acceleration.x / 9.81f; 
    float y = accel.acceleration.y / 9.81f;
    float z = accel.acceleration.z / 9.81f;
    // 3D vector magnitude  (Pythagoren theorem)
    return sqrtf(x*x + y*y + z*z);
}

// ── Public API 

void fallDetectInit() {
    fallConfirmed   = false;
    freeFallSeen    = false;
    freeFallTimeMs  = 0;
    freeFallEntryMs = 0;
    lastFallMs      = 0;
    sensorReady     = false;

    // Tries to connect to the IMU chip over I2C, if it can't find it, it prints an error and stops
    if (!imu.begin_I2C(LSM_I2C_ADDR, &Wire)) {
        Serial.println("[FALL] LSM6DSOX not found — check wiring");
        return;
    }

    Serial.println("[FALL] LSM6DSOX found — configuring");

    // ──  Writes to the sensor's internal configuration registers
    // Sets the sensor to sample at 416 times per second
    writeReg(0x10, 0x60);   // ODR 416Hz
    // Configures the hardware free-fall detector
    writeReg(0x5D, 0x21);   // free-fall threshold 156mg, 4 samples
    writeReg(0x5E, 0x10);   // route to INT1

    // Configures the ESP32 pin connected to INT1 as an input with a pullup resistor
    pinMode(LSM_INT1_PIN, INPUT_PULLUP);

    sensorReady = true;
    Serial.println("[FALL] Fall detection active");
}

void fallDetectUpdate() {
    if (!sensorReady) return;

    uint32_t now = millis();
    static uint32_t lastFallSampleMs = 0;

    // Only runs every 10ms, saves processing power
    if ((now - lastFallSampleMs) < 10) return;
    lastFallSampleMs = now;

    // If a fall was already confirmed but not yet acknowledged
    // Or if we're still in cooldown from the last fall, skip everything
    if (fallConfirmed || (now - lastFallMs) <= FALL_COOLDOWN_MS) return;

    float raw = getRawMagnitude();

    if (!freeFallSeen) { 
        // Stage 1 — Free-fall Detection
        // If the total g-force drops below 0.4g, start a timer
        // If it stays below 0.4g for at least Free_Fall_MIN-MS (80ms), free-fall is confirmed
        // If it pops back above 0.4g beform 80ms, it was just a wrist shake, reset the timer
        if (raw < 0.4f) {
            if (freeFallEntryMs == 0) {
                freeFallEntryMs = now; // start timing
                Serial.print("[FALL] Free-fall candidate — raw: ");
                Serial.print(raw, 3);
                Serial.println("g");
            } else if ((now - freeFallEntryMs) >= FREE_FALL_MIN_MS) {
                freeFallSeen    = true; // weightless long enough - confirmed
                freeFallTimeMs  = now;
                freeFallEntryMs = 0;
                Serial.print("[FALL] Free-fall confirmed — raw: ");
                Serial.print(raw, 3);
                Serial.println("g");
            }
        } else {
            if (freeFallEntryMs != 0) {
                Serial.println("[FALL] Free-fall rejected — too brief (shake)");
                freeFallEntryMs = 0; // wasn't sustained, probably a wrist shake, reset
            }
        }
        return;
    }

    // Free-fall Windo Expiry
    // After free-fall is confirmed, the code waits up to 600ms for an impact
    // If no impact comes in that window, maybe someone caught themselves, it resets
    if ((now - freeFallTimeMs) > 600) {
        freeFallSeen = false;
        Serial.println("[FALL] Free-fall window expired — no impact");
        return;
    }

    // Stage 2 — Impact Detection
    // If after a confirmed free-fall the g-force suddenly spikes above 2.5g, fall confirmed
    if (raw > 2.5f) {
        freeFallSeen  = false;
        fallConfirmed = true;
        lastFallMs    = now;

        Serial.print("[FALL] Impact detected — raw: ");
        Serial.print(raw, 3);
        Serial.println("g — FALL CONFIRMED");
    }
}

// ── Getters and acknowledgements 
bool fallDetectFallConfirmed() {
    return fallConfirmed;
}

void fallDetectAcknowledge() {
    fallConfirmed = false;
    Serial.println("[FALL] Fall acknowledged");
}

// ── Register write helper 
static void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LSM_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}