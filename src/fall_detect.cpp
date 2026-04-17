#include "fall_detect.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

// ── Sensor instance ────────────────────────────────────────────────────────
static Adafruit_LSM6DSOX imu;
static bool     sensorReady       = false;

// ── Interrupt state ────────────────────────────────────────────────────────
// Marked volatile — written by ISR on different task than loop()
static volatile bool isrFired     = false;

// ── Detection state ────────────────────────────────────────────────────────
static bool     fallConfirmed     = false;
static bool     pendingConfirm    = false;   // interrupt fired, awaiting confirm window
static uint32_t pendingStartMs    = 0;
static uint32_t lastFallMs        = 0;

// ── ISR — called by hardware when INT1 goes low ────────────────────────────
static void IRAM_ATTR onFallInterrupt() {
    isrFired = true;
}

// ── Direct register write via Wire ────────────────────────────────────────
// Used for registers not exposed by the Adafruit LSM6DSOX class API
static void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LSM_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// ── Public API ─────────────────────────────────────────────────────────────

void fallDetectInit() {
    fallConfirmed  = false;
    pendingConfirm = false;
    isrFired       = false;
    lastFallMs     = 0;
    sensorReady    = false;

    // Wire already started by heartMonitorInit()
    if (!imu.begin_I2C(LSM_I2C_ADDR, &Wire)) {
        Serial.println("[FALL] LSM6DSOX not found — check wiring");
        return;
    }

    Serial.println("[FALL] LSM6DSOX found — configuring free-fall detection");

    // ── Set accelerometer ODR to 416Hz via Wire directly ──────────────────
    // CTRL1_XL register (0x10): ODR_XL = 0110 (416Hz), FS_XL = 00 (2g)
    // Value: 0b01100000 = 0x60
    writeReg(0x10, 0x60);

    // ── Free-fall threshold and duration ──────────────────────────────────
    // FREE_FALL register (0x5D):
    // Bits [2:0] = FF_THS: 0x02 = 250mg threshold
    // Bits [7:3] = FF_DUR: 4 samples = 0x04 << 3 = 0x20
    // Combined: 0x20 | 0x02 = 0x22
    writeReg(0x5D, 0x22);

    // ── Route free-fall to INT1 ────────────────────────────────────────────
    // MD1_CFG register (0x5E): bit 4 = INT1_FF
    writeReg(0x5E, 0x10);

    // ── GPIO interrupt ─────────────────────────────────────────────────────
    pinMode(LSM_INT1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LSM_INT1_PIN),
                    onFallInterrupt,
                    FALLING);

    sensorReady = true;
    Serial.println("[FALL] Free-fall detection active on GPIO" +
                   String(LSM_INT1_PIN));
}

void fallDetectUpdate() {
    if (!sensorReady) return;

    uint32_t now = millis();

    // ── Check cooldown ─────────────────────────────────────────────────────
    if (fallConfirmed && (now - lastFallMs) < FALL_COOLDOWN_MS) return;

    // ── ISR fired — start confirmation window ─────────────────────────────
    if (isrFired && !pendingConfirm) {
        isrFired       = false;
        pendingConfirm = true;
        pendingStartMs = now;
        Serial.println("[FALL] Interrupt received — confirming...");
        return;
    }

    // ── Confirmation window — wait FALL_CONFIRM_MS then confirm ───────────
    // This filters out brief vibrations that might trigger the hardware detector
    if (pendingConfirm && (now - pendingStartMs) >= FALL_CONFIRM_MS) {
        pendingConfirm = false;
        fallConfirmed  = true;
        lastFallMs     = now;

        Serial.println("[FALL] Fall confirmed");

        // Notify heart monitor to suppress readings during motion artifact window
        // This is called here so fall_detect stays decoupled from heart_monitor
        // fusion_logic will also call heartMonitorNotifyFall() — belt and braces
    }
}

bool fallDetectFallConfirmed() {
    return fallConfirmed;
}

void fallDetectAcknowledge() {
    fallConfirmed = false;
    Serial.println("[FALL] Fall acknowledged — ready for next detection");
}

// ── Simulation only ────────────────────────────────────────────────────────
void fallDetectSimulateInterrupt() {
    isrFired = true;
    Serial.println("[FALL] Simulated interrupt fired");
}