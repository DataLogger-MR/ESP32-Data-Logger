#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ================ DOUBLE BUFFER GLOBALS ================
extern char* buffers[2];
extern int activeBufferIndex;
extern int bufferIndex;
extern SemaphoreHandle_t bufferMutex;
extern QueueHandle_t flushQueue;
extern SemaphoreHandle_t bufferFreeSem[2];

// ================ EXISTING GLOBALS ================
extern unsigned long loggedCount;
extern unsigned long messageCount;
extern unsigned long acceptedCount;
extern unsigned long filteredOutCount;
extern unsigned long startTime;

// ================ FUNCTION PROTOTYPES ================
void initDataLogger();
void logDataToSD();
void flushBuffer();               
void addToBuffer(const char* data, int len);
void formatCompleteCSV(char* buffer, size_t bufferSize, 
                       unsigned long timestampMs, unsigned long currentTime);
void resetStatistics();
void setDynamicMode(bool enable);
void updateDynamicHeader();
String getDynamicHeader();
void logDynamicDataToSD();

void flushBufferTask(void *pvParameters);

#endif