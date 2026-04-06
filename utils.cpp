#include "utils.h"
#include <sys/time.h>   // for gettimeofday

uint32_t getBits(const uint8_t* data, int startBit, int length) {
  uint32_t result = 0;
  for (int i = 0; i < length; i++) {
    if (getBit(data, startBit + i)) {
      result |= (1 << i);
    }
  }
  return result;
}

bool isValid(unsigned long lastUpdate, unsigned long timeoutMs, unsigned long currentTime) {
  if (lastUpdate == 0) return false; // Never received
  if (timeoutMs == 0) return true; // No timeout
  return (currentTime - lastUpdate) <= timeoutMs;
}

String formatBytes(size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else return String(bytes / 1048576.0) + " MB";
}

String getFormattedTime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    struct tm *timeinfo = localtime(&now);
    if (timeinfo->tm_year < 100) {   // time not yet set
        return String(millis());      // fallback
    }
    char buffer[30];
    sprintf(buffer, "%02d:%02d:%02d.%03ld",
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
            tv.tv_usec / 1000);
    return String(buffer);
}

String formatEpochToLocal(time_t epoch) {
    struct tm *tm_info = localtime(&epoch);
    char buffer[20];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tm_info);
    return String(buffer);
}