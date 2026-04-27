#include "sd_card.h"
#include "utils.h"
#include "globals.h" 

bool sdReady = false;

void initSD() {
  Serial.print("Initializing SD card... ");
  
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  delay(100);
  
  if (!SD.begin(SD_CS)) {
    Serial.println("❌ FAILED!");
    sdReady = false;
    return;
  }
  
  Serial.println("✅ OK!");
  sdReady = true;
  
  getSDInfo();
}

void getSDInfo() {
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  uint64_t totalSize = SD.totalBytes() / (1024 * 1024);
  uint64_t usedSize = SD.usedBytes() / (1024 * 1024);
  
}

bool createDirectory(const char* path) {
  if (!sdReady) return false;
  return SD.mkdir(path);
}

bool createDirectoryRecursive(const char* path) {
  if (!sdReady) return false;
  
  char tempPath[128];
  strcpy(tempPath, path);
  
  for (char* p = tempPath + 1; *p; p++) {
    if (*p == '/') {
      *p = '\0';
      if (!SD.exists(tempPath)) {
        if (!SD.mkdir(tempPath)) {
          Serial.printf("Failed to create directory: %s\n", tempPath);
          return false;
        }
      }
      *p = '/';
    }
  }
  
  if (!SD.exists(tempPath)) {
    if (!SD.mkdir(tempPath)) {
      Serial.printf("Failed to create directory: %s\n", tempPath);
      return false;
    }
  }
  return true;
}

bool fileExists(const char* path) {
  if (!sdReady) return false;
  return SD.exists(path);
}

uint64_t getSDFreeSpace() {
  if (!sdReady) return 0;
  return SD.totalBytes() - SD.usedBytes();
}

uint64_t getSDTotalSpace() {
  if (!sdReady) return 0;
  return SD.totalBytes();
}

void printSDInfo() {
  if (!sdReady) {
    Serial.println("SD card not ready");
    return;
  }
  
  uint64_t total = SD.totalBytes() / (1024 * 1024);
  uint64_t used = SD.usedBytes() / (1024 * 1024);
  
  Serial.printf("SD Card: %llu MB total, %llu MB used, %llu MB free\n",
                total, used, total - used);
}