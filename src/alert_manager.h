#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── Alert types 
#define ALERT_GUNSHOT      "GUNSHOT"
#define ALERT_UNCONSCIOUS  "UNCONSCIOUSNESS"
#define ALERT_FALL         "FALL"
#define ALERT_NONE         "NONE"

// ── Confidence tiers 
#define CONFIDENCE_LOW    "LOW"
#define CONFIDENCE_MEDIUM "MEDIUM"
#define CONFIDENCE_HIGH   "HIGH"

// ── Public API 
void alertManagerInit();
void alertManagerUpdate();
bool alertManagerIsConnected();

void alertManagerSendGunshot(const char* confidence);
void alertManagerSendFall(const char* confidence);
void alertManagerSendUnconscious(const char* confidence);

// Sends any raw JSON string directly — no cooldown, no co-occurrence check
// Used for live data broadcasts like BPM during testing
void alertManagerSendRaw(const char* payload);