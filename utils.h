#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "types.h"
#include "config.h"  

// ================ DEBUG MACROS ================
#ifdef ENABLE_SERIAL_DEBUG
    #if ENABLE_SERIAL_DEBUG
        #define DEBUG_PRINT(x) Serial.print(x)
        #define DEBUG_PRINTLN(x) Serial.println(x)
        #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
    #else
        #define DEBUG_PRINT(x)
        #define DEBUG_PRINTLN(x)
        #define DEBUG_PRINTF(...)
    #endif
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif

// ================ DATA EXTRACTION HELPERS ================

inline uint16_t getIntel16(const uint8_t* data, int startByte) {
  return data[startByte] | (data[startByte + 1] << 8);
}

inline uint32_t getIntel32(const uint8_t* data, int startByte) {
  return data[startByte] | (data[startByte + 1] << 8) | 
         (data[startByte + 2] << 16) | (data[startByte + 3] << 24);
}

inline uint16_t getMotorola16(const uint8_t* data, int startByte) {
  return (data[startByte] << 8) | data[startByte + 1];
}

inline uint32_t getMotorola32(const uint8_t* data, int startByte) {
  return (data[startByte] << 24) | (data[startByte + 1] << 16) | 
         (data[startByte + 2] << 8) | data[startByte + 3];
}

inline bool getBit(const uint8_t* data, int bitPos) {
  int byteIndex = bitPos / 8;
  int bitInByte = bitPos % 8;
  return (data[byteIndex] >> bitInByte) & 1;
}

uint32_t getBits(const uint8_t* data, int startBit, int length);

bool isValid(unsigned long lastUpdate, unsigned long timeoutMs, unsigned long currentTime);

String formatBytes(size_t bytes);
String getFormattedTime();
String formatEpochToLocal(time_t epoch);

#endif