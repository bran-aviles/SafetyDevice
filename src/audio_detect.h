#pragma once
#include <stdint.h>   // int32_t — works in both Arduino and test environments
#include <stdbool.h>  // bool

// ── Public API
void audioDetectInit();                              // call once in setup()
bool audioDetectUpdate();                            // call every loop()

void audioDetectNotifyFall();   // suppresses audio detection for 3 seconds

void  resetDetection();
void  processSample(int32_t sample);
bool  runAudioDetect(int32_t* buffer, int bufferSize);