#include "audio_detect.h"
#include "config.h"

// ── State ──────────────────────────────────────────────────────────────────
static bool    onsetDetected        = false;
static bool    flatlineDetected     = false;
static int     flatlineCount        = 0;
static int     samplesSinceFlatline = 0;
static int32_t prevSample           = 0;
static bool    audioEventDetected   = false;

// ── Internal ───────────────────────────────────────────────────────────────
void resetDetection() {
    onsetDetected        = false;
    flatlineDetected     = false;
    flatlineCount        = 0;
    samplesSinceFlatline = 0;
    prevSample           = 0;
}

// ── Process a single sample through the detector ──
void processSample(int32_t sample) {
    // Stage 1: look for onset — sharp rise into clipping
    if (!onsetDetected) {
        int32_t diff = sample - prevSample;
        if (diff > ONSET_DIFF && sample >= CLIP_THRESHOLD) {
            onsetDetected = true;
        }
        prevSample = sample;
        return;
    }

    // Stage 2: count flatline (clipped) samples
    if (onsetDetected && !flatlineDetected) {
        if (sample >= CLIP_THRESHOLD) {
            flatlineCount++;

            // Too long — probably sustained noise not a gunshot
            if (flatlineCount > MAX_FLATLINE) {
                resetDetection();   
                return;
            }
        } else {
             // Signal dropped before minimum flatline — not valid
            if (flatlineCount < MIN_FLATLINE) {
                resetDetection();   
                return;
            }
            // Valid flatline duration confirmed
            flatlineDetected = true;
        }
        return;
    }

    // Stage 3: wait for decay below threshold after flatline ends
    if (flatlineDetected) {
        samplesSinceFlatline++;
        if (sample < DECAY_THRESHOLD) {
            audioEventDetected = true;
            resetDetection();
            return;
        }
        if (samplesSinceFlatline > DECAY_WINDOW) {
            resetDetection();   // decay never came — not a gunshot
        }
    }
}

// ── Run detector on a buffer of samples ──
bool runAudioDetect(int32_t* buffer, int bufferSize) {
    audioEventDetected = false;
    resetDetection();
    for (int i = 0; i < bufferSize; i++) {
        // SPH0645 raw I2S value: 24-bit packet, only 18 bits meaningful,
        // lower 6 bits are zero padding — shift before comparing thresholds
        int32_t trueSample = buffer[i] >> 6;
        processSample(trueSample);
        if (audioEventDetected) return true;
    }
    return false;
}