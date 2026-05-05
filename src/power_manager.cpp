#include "power_manager.h"
#include "config.h"
#include "alert_manager.h"
#include <Arduino.h>
#include <esp_sleep.h> // ESP32's built in sleep mode library - controls putting the chip to sleep and waking it up
#include <driver/gpio.h> // low level GPIO pin control

// ── Power button state 
#define POWER_BTN_PIN           7       // IO7 — connected to GND via button
#define POWER_BTN_HOLD_MS       1000    // hold 1s to trigger sleep/wake
#define DEBOUNCE_MS             50

static uint32_t btnPressStartMs = 0;
static bool     btnHeld         = false;
static bool     deviceAwake     = true;

// ── Enter deep sleep 
// Configures IO7 as a wake source then enters deep sleep.
// Current draw in deep sleep: ~5–10µA on ESP32-C3.
// With PKCell LP503562 1200mAh at 10µA: theoretical standby = ~13 years.
// Real standby accounting for leakage and BLE init on wake: weeks to months.
static void enterLightSleep() {
    Serial.println("[PWR] Entering light sleep — press button to wake");

    // Notify BLE client before sleeping so the app can update its UI
    if (alertManagerIsConnected()) {
        alertManagerSendRaw("{\"type\":\"DEVICE_SLEEP\"}");
        delay(100);   // give BLE stack time to transmit before sleeping
    }

    alertManagerLEDOff();    // ← ADD THIS — turn LED off before sleeping

    Serial.flush();
    delay(200);

    gpio_wakeup_enable((gpio_num_t)POWER_BTN_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();

    // ── Execution resumes here after wake 
    Serial.println("[PWR] Woke from light sleep via power button");

    // Notify BLE client that device is awake again
    if (alertManagerIsConnected()) {
        alertManagerSendRaw("{\"type\":\"DEVICE_WAKE\"}");
    }

    // Wait for button release before resuming
    while (digitalRead(POWER_BTN_PIN) == LOW) {
        delay(10);
    }
    delay(DEBOUNCE_MS);
}

// ── Public API 
void powerManagerInit() {
    pinMode(POWER_BTN_PIN, INPUT_PULLUP);

    // Light sleep resumes execution rather than rebooting,
    // so no wakeup cause check needed here
    Serial.println("[PWR] Hold button 1s to sleep, press to wake");
    deviceAwake     = true;
    btnPressStartMs = 0;
    btnHeld         = false;
}

void powerManagerUpdate() {
    uint32_t now    = millis();
    bool     btnLow = (digitalRead(POWER_BTN_PIN) == LOW);

    if (btnLow) {
        if (btnPressStartMs == 0) {
            // Button just pressed — start timing
            btnPressStartMs = now;
        } else if (!btnHeld && (now - btnPressStartMs) >= POWER_BTN_HOLD_MS) {
            // Held long enough — trigger sleep
            btnHeld = true;
            Serial.println("[PWR] Power button held — going to sleep");
            enterLightSleep();
        }
    } else {
        // Button released — reset state
        if (btnPressStartMs != 0 && !btnHeld) {
            // Short press — ignore (only long hold triggers sleep)
            Serial.println("[PWR] Short press — hold 1s to sleep");
        }
        btnPressStartMs = 0;
        btnHeld         = false;
    }
}

bool powerManagerIsAwake() {
    return deviceAwake;
}