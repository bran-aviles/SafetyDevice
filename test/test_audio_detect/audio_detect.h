#pragma once
#include <stdint.h>   // int32_t — works in both Arduino and test environments
#include <stdbool.h>  // bool

void  resetDetection();
void  processSample(int32_t sample);
bool  runAudioDetect(int32_t* buffer, int bufferSize);