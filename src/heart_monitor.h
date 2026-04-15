#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── Public API ─────────────────────────────────────────────────────────────

// Call once in setup()
void heartMonitorInit();

// Call every loop() — handles timing internally
void heartMonitorUpdate();

// Called by fusion logic to get current state
uint8_t  heartMonitorGetBPM();
bool     heartMonitorBaselineReady();
bool     heartMonitorUnconsciousFlag();

// Called by fall_detect when impact is confirmed
void heartMonitorNotifyFall();

// Called by audio_detect when gunshot event fires
void heartMonitorNotifyAudioEvent();

// Path A result — for fusion logic
// Returns  1 if HR changed after audio event (supports gunshot)
// Returns -1 if HR completely flat after audio event (reduces gunshot confidence)
// Returns  0 if no audio event active or window expired
int8_t heartMonitorGetPathAResult();