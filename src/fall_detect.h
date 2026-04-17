#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── Public API 

// Call once in setup()
void fallDetectInit();

// Call every loop() — handles confirmation and cooldown
void fallDetectUpdate();

// Returns true if a fall was confirmed and not yet acknowledged
bool fallDetectFallConfirmed();

// Call after fusion logic has processed the fall — clears the flag
void fallDetectAcknowledge();

// Raw interrupt flag — set by ISR, read by fallDetectUpdate()
// Exposed so test harness can simulate an interrupt
void fallDetectSimulateInterrupt();