#include "audio_detect.h"
#include "config.h"
#include <Arduino.h>
#include <driver/i2s.h>

// Description
// Reads audio from the the digital mic as a stream of 32-bit integers
// Sampled 32,000 times per second, every 8ms it grabs 256 of these integers
// Scans them looking for a gunshot's unique numerical fingerprint
// Checks for a sudden spike(fast onset), Brief maxing out of the mic(flatline/clipping), Quick drop-off in volume (decay)


// ── Detection state
static bool    onsetDetected        = false; 
static bool    flatlineDetected     = false;
static int     flatlineCount        = 0;
static int     samplesSinceFlatline = 0; // Variable for how long has it been since the clip ended
static int32_t prevSample           = 0; 
static bool    audioEventDetected   = false;
static bool     audioFallSuppress      = false;
static uint32_t audioFallSuppressStart = 0;


// ── Reset detection state
void resetDetection() {
    onsetDetected        = false;
    flatlineDetected     = false;
    flatlineCount        = 0;
    samplesSinceFlatline = 0;
    prevSample           = 0;
}

// ── I2S read buffer 
// Chunk of memory that holds a batch of raw audio samples
static int32_t i2sBuffer[I2S_BUFFER_SIZE];

// ── Process a single sample through the detector
void processSample(int32_t sample) {

    // Stage 1: look for fast onset — sharp rise into clipping
    // Compared the current audio sample to the previous one
    // If the volume jumped up very fast and is already extremely loud
    if (!onsetDetected) {
        int32_t diff = sample - prevSample;
        if (diff > ONSET_DIFF && sample >= CLIP_THRESHOLD) {
            onsetDetected = true;
        }
        prevSample = sample;
        return;
    }

    // Stage 2: count flatline (clipped) samples
    // Checks if the sound max clipped the microphone for a brief moment 
    if (onsetDetected && !flatlineDetected) {
        if (sample >= CLIP_THRESHOLD) {
            flatlineCount++;

            // Too long — probably sustained noise not a gunshot
            if (flatlineCount > MAX_FLATLINE) {
                resetDetection();   
                return;
            }
        } else {
             // Signal dropped before minimum flatline 
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
    // Waits to see the volume fall back down 
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

// ── Run detector on a buffer of samples 
// Loops through every sample in the audio bucket
// Feeds them one by one into processSample()
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


// ── I2S init 
// Sets up the I2S microphone
// Configures the sample rate, bit depth, pins to use
void audioDetectInit() {
    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = I2S_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 4,
        .dma_buf_len          = I2S_BUFFER_SIZE,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num   = I2S_BCLK,
        .ws_io_num    = I2S_LRCL,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_DOUT
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);

    Serial.println("[AUDIO] SPH0645 I2S ready");
}

// ── I2S update — call every loop() 
bool audioDetectUpdate() {
    // ── Fall suppression — ignore audio after fall impact 
    if (audioFallSuppress) {
        if ((millis() - audioFallSuppressStart) < AUDIO_FALL_SUPPRESS_MS) {
            size_t bytesRead = 0;
            i2s_read(I2S_PORT, i2sBuffer, sizeof(i2sBuffer), &bytesRead, 0);
            return false;   // drain buffer but don't detect
        }
        audioFallSuppress = false;
        Serial.println("[AUDIO] Fall suppression ended — audio detection resumed");
    }
    
    size_t bytesRead = 0;

    // Reads a fresh batch of audio samples from the mic into the buffer
    i2s_read(I2S_PORT, i2sBuffer, sizeof(i2sBuffer), &bytesRead, 0);
    if (bytesRead == 0) return false;

    int samplesRead = bytesRead / sizeof(int32_t);

    // ── Debug — print largest onset diff seen in this buffer ──────────────
    int32_t maxDiff = 0;
    int32_t prev    = 0;
    for (int i = 0; i < samplesRead; i++) {
        int32_t sample = i2sBuffer[i] >> 6;
        int32_t diff   = sample - prev;
        if (diff > maxDiff) maxDiff = diff;
        prev = sample;
    }
    if (maxDiff > 150000) {
        Serial.print("[AUDIO] Max onset diff: ");
        Serial.println(maxDiff);
    }
    // ── End debug 
    
    // Passes the fresh read audio into the detector and returns whether a gunshot was found
    return runAudioDetect(i2sBuffer, samplesRead);
}

// Ignores loud sounds for a few secs after someone just fell
void audioDetectNotifyFall() {
    audioFallSuppress      = true;
    audioFallSuppressStart = millis();
    Serial.println("[AUDIO] Fall notified — suppressing audio detection 3 seconds");
}