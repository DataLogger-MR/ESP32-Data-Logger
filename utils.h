#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "types.h"
#include "config.h"  // Include for ENABLE_SERIAL_DEBUG

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

// Intel (Little Endian) 16-bit
inline uint16_t getIntel16(const uint8_t* data, int startByte) {
  return data[startByte] | (data[startByte + 1] << 8);
}

// Intel (Little Endian) 32-bit
inline uint32_t getIntel32(const uint8_t* data, int startByte) {
  return data[startByte] | (data[startByte + 1] << 8) | 
         (data[startByte + 2] << 16) | (data[startByte + 3] << 24);
}

// Motorola (Big Endian) 16-bit
inline uint16_t getMotorola16(const uint8_t* data, int startByte) {
  return (data[startByte] << 8) | data[startByte + 1];
}

// Motorola (Big Endian) 32-bit
inline uint32_t getMotorola32(const uint8_t* data, int startByte) {
  return (data[startByte] << 24) | (data[startByte + 1] << 16) | 
         (data[startByte + 2] << 8) | data[startByte + 3];
}

// Get bit from specific position
inline bool getBit(const uint8_t* data, int bitPos) {
  int byteIndex = bitPos / 8;
  int bitInByte = bitPos % 8;
  return (data[byteIndex] >> bitInByte) & 1;
}

// Get bits from specific range
uint32_t getBits(const uint8_t* data, int startBit, int length);

// Check if data is still valid based on timeout
bool isValid(unsigned long lastUpdate, unsigned long timeoutMs, unsigned long currentTime);

// Format bytes to human readable string
String formatBytes(size_t bytes);
// NEW
String getFormattedTime();
String formatEpochToLocal(time_t epoch);

#endif