#ifndef SD_CARD_H
#define SD_CARD_H

#include <Arduino.h>
#include <SD.h>
#include "pins.h"

// ================ SD CARD GLOBALS ================
extern bool sdReady;

// ================ FUNCTION PROTOTYPES ================
void initSD();
void getSDInfo();
bool createDirectory(const char* path);
bool createDirectoryRecursive(const char* path);
bool fileExists(const char* path);
uint64_t getSDFreeSpace();
uint64_t getSDTotalSpace();
void printSDInfo();

#endif