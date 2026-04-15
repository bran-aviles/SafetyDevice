#include "alert_manager.h"
#include "config.h"
#include "audio_detect.h"
#include "heart_monitor.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>     

// ── NeoPixel instance 
static Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── LED pulse state 
static uint32_t lastPulseMs    = 0;
static uint8_t  pulseDirection = 1;     // 1 = brightening, 0 = dimming
static uint8_t  pulseBrightness = 0;

// ── BLE handles ─
BLEServer*         pServer               = NULL;
BLECharacteristic* pAlertCharacteristic  = NULL;


// ── Cooldown tracking ──────────────────────────────────────────────────────
static unsigned long lastGunshotAlert    = 0;
static unsigned long lastFallAlert       = 0;
static unsigned long lastUnconsciousAlert = 0;

// ── Co-occurrence tracking ─────────────────────────────────────────────────
static String        lastFiredType = ALERT_NONE;
static unsigned long lastFiredTime = 0;

// ── BLE connection callbacks ───────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("[BLE] Device connected");
    }
    void onDisconnect(BLEServer* pServer) {
        Serial.println("[BLE] Device disconnected — restarting advertising");
        BLEDevice::startAdvertising();
    }
};

// ── Build JSON payload
static String buildPayload(String alertType, String confidence, String coOccurrence) {
    String payload = "{";
    payload += "\"type\":\""         + String(alertType)   + "\",";
    payload += "\"confidence\":\""   + String(confidence)  + "\",";
    payload += "\"co_occurrence\":\"" + String(coOccurrence) + "\"";
    payload += "}";
    return payload;
}

// ── Co-occurrence check
static String checkCoOccurrence(String currentType) {
    unsigned long now = millis();
    if (lastFiredType != ALERT_NONE &&
        lastFiredType != currentType &&
        (now - lastFiredTime) <= CO_OCCURRENCE_WINDOW_MS) {
        return lastFiredType;
    }
    return ALERT_NONE;
}

// ── Send alert ─────────────────────────────────────────────────────────────
static void sendAlert(String alertType, String confidence, unsigned long &lastAlertTime) {
    // Connection check using server directly — avoids FreeRTOS flag race condition
    if (pServer->getConnectedCount() == 0) {
        Serial.println("[BLE] Not connected — alert suppressed");
        return;
    }

    unsigned long now = millis();

    // Cooldown check
    if ((now - lastAlertTime) < COOLDOWN_MS) {
        Serial.println("[BLE] Cooldown active for " + alertType + " — suppressed");
        return;
    }

    // Co-occurrence check
    String coOccurrence = checkCoOccurrence(alertType);

    // Build and send
    String payload = buildPayload(alertType, confidence, coOccurrence.c_str());
    pAlertCharacteristic->setValue(payload.c_str());
    pAlertCharacteristic->notify();

    // Update tracking
    lastAlertTime  = now;
    lastFiredType  = alertType;
    lastFiredTime  = now;

    Serial.println("[BLE] Alert sent: " + payload);
}

// ── Public API ─────────────────────────────────────────────────────────────

void alertManagerInit() {
// ── LED init
    led.begin();
    led.setBrightness(LED_BRIGHTNESS);
    led.clear();
    led.show();

    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(100);     

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pAlertCharacteristic = pService->createCharacteristic(
        CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pAlertCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Ready — waiting for connection");
}

void alertManagerUpdate() {
      uint32_t now = millis();

    if (alertManagerIsConnected()) {
        // ── Solid blue when connected ──────────────────────────────────────
        led.setPixelColor(0, led.Color(0, 0, 255));
        led.show();
    } else {
        // ── Slow pulse while waiting for connection ────────────────────────
        // Pulse period: ~2 seconds up, ~2 seconds down
        if (now - lastPulseMs >= 20) {   // update brightness every 20ms
            lastPulseMs = now;

            if (pulseDirection == 1) {
                pulseBrightness += 3;
                if (pulseBrightness >= 150) {
                    pulseBrightness = 150;
                    pulseDirection  = 0;
                }
            } else {
                if (pulseBrightness <= 3) {
                    pulseBrightness = 0;
                    pulseDirection  = 1;
                } else {
                    pulseBrightness -= 3;
                }
            }

            led.setPixelColor(0, led.Color(0, 0, pulseBrightness));
            led.show();
        }
    }
}

bool alertManagerIsConnected() {
     return pServer->getConnectedCount() > 0;
}

void alertManagerSendGunshot(const char* confidence) {
    sendAlert(ALERT_GUNSHOT, confidence, lastGunshotAlert);
}

void alertManagerSendFall(const char* confidence) {
    sendAlert(ALERT_FALL, confidence, lastFallAlert);
}

void alertManagerSendUnconscious(const char* confidence) {
    sendAlert(ALERT_UNCONSCIOUS, confidence, lastUnconsciousAlert);
}

void alertManagerSendRaw(const char* payload) {
    pAlertCharacteristic->setValue(payload);
    pAlertCharacteristic->notify();
    Serial.println("[BLE] Raw sent: " + String(payload));
}