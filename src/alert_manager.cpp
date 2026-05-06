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

// ── NeoPixel LED object
static Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── LED pulse state 
static uint32_t lastPulseMs     = 0; 
static uint8_t  pulseDirection  = 1; 
static uint8_t  pulseBrightness = 0; 

// ── BLE handles 
// BLEServer* acts as the server to hold data, adverstising for a central device (client)
BLEServer*         pServer              = NULL;
// BLECharactersistic* manages specific data values (sensor readings) on a server, sends alert messages
BLECharacteristic* pAlertCharacteristic = NULL;

static bool ledConnectedSet = false;

// ── Cooldown tracking 
static unsigned long lastGunshotAlert   = 0;
static unsigned long lastFallAlert      = 0;

// ── BLE connection callbacks
// watches for phones connecting or disconnecting via BLE 
class ServerCallbacks : public BLEServerCallbacks {
    // prints "device connected"
    void onConnect(BLEServer* pServer) {
        Serial.println("[BLE] Device connected");
    }
    // resets the LED flag and restarts Bluetooth adverstising
    void onDisconnect(BLEServer* pServer) {
        ledConnectedSet = false;
        Serial.println("[BLE] Device disconnected — restarting advertising");
        BLEDevice::startAdvertising();
    }
};

// ── Build JSON payload 
// Packages the alert into a JSON string(standard text format)
static String buildPayload(String alertType, String confidence) {
    String payload = "{";
    payload += "\"type\":\""       + alertType  + "\",";
    payload += "\"confidence\":\"" + confidence + "\"";
    payload += "}";
    return payload;
}

// ── Send alert 
static void sendAlert(String alertType, String confidence,
                      unsigned long &lastAlertTime) {
    // Checks if anyone has connected via Bluetooth, if not don't send alert
    if (pServer->getConnectedCount() == 0) { 
        Serial.println("[BLE] Not connected — alert suppressed");
        return;
    }

    unsigned long now = millis();

    // Cooldown for alert sending
    if ((now - lastAlertTime) < COOLDOWN_MS) {
        Serial.print("[BLE] Cooldown active for ");
        Serial.print(alertType);
        Serial.print(" — ");
        Serial.print((COOLDOWN_MS - (now - lastAlertTime)) / 1000);
        Serial.println("s remaining");
        return;
    }

    String payload = buildPayload(alertType, confidence);
    // Loads the alert message into the Bluetooth channel
    pAlertCharacteristic->setValue(payload.c_str());

    // Check if any client has notifications enabled before notifying
    // BLE2902 descriptor holds the notification enable flag per client
    BLE2902* desc = (BLE2902*)pAlertCharacteristic->getDescriptorByUUID(
                        BLEUUID((uint16_t)0x2902));
    bool notificationsEnabled = desc != nullptr && desc->getNotifications();

    Serial.print("[BLE] Sending alert: ");
    Serial.println(payload);
    Serial.print("[BLE] Connected clients: ");
    Serial.println(pServer->getConnectedCount());
    Serial.print("[BLE] Notifications enabled: ");
    Serial.println(notificationsEnabled ? "YES" : "NO  ← client must subscribe first");

    // Sends the alert and records the time so the cooldown can kick in
    pAlertCharacteristic->notify();
    lastAlertTime = now;

    Serial.println("[BLE] Notify called — alert sent");
}

// ── Public API 
// Turns on the LED and clears it
// Initializes Bluetooth with a device name
// Creates the Bluetooth server and alert channel
// Starts advertising 
void alertManagerInit() {
    led.begin();
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


    // ADD THESE TWO LINES before startAdvertising()
    pAdvertising->setMinInterval(1600);   // 500ms  (units are 0.625ms, so 800 × 0.625 = 500ms)
    pAdvertising->setMaxInterval(1600);

    BLEDevice::startAdvertising();

    Serial.println("[BLE] Ready — waiting for connection");
}

void alertManagerUpdate() {
    uint32_t now = millis();

    if (alertManagerIsConnected()) {
        // Turn LED solid blue once connected to BLE
        if (!ledConnectedSet) {
            led.setPixelColor(0, led.Color(0, 0, 60));
            led.show();
            ledConnectedSet = true;
        }
    } else {
        // Slowly pulse the LED blue (breathing effect)
        ledConnectedSet = false;

        // Slow pulse while waiting for connection
        if (now - lastPulseMs >= 20) {
            lastPulseMs = now;

            if (pulseDirection == 1) {
                pulseBrightness += 1;
                if (pulseBrightness >= 30) {
                    pulseBrightness = 30;
                    pulseDirection  = 0;
                }
            } else {
                if (pulseBrightness <= 3) {
                    pulseBrightness = 0;
                    pulseDirection  = 1;
                } else {
                    pulseBrightness -= 1;
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

// Sends a gunshot alert with a confidence level 
void alertManagerSendGunshot(const char* confidence) {
    sendAlert(ALERT_GUNSHOT, confidence, lastGunshotAlert);
}

// Sends a fall-detect alert with a confidence level
void alertManagerSendFall(const char* confidence) {
    sendAlert(ALERT_FALL, confidence, lastFallAlert);
}

// Sends any custom message directly
void alertManagerSendRaw(const char* payload) {
    pAlertCharacteristic->setValue(payload);
    pAlertCharacteristic->notify();
    Serial.println("[BLE] Raw sent: " + String(payload));
}

void alertManagerLEDOff() {
    led.clear();
    led.show();
}