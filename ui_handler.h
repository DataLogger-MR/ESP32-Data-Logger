#ifndef UI_HANDLER_H
#define UI_HANDLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ================ UI COMMAND BUFFER ================
extern char uiCommandBuffer[128];
extern int uiCommandIndex;

// ================ Mutex for Serial1 ================
extern SemaphoreHandle_t serial1Mutex;

// ================ FUNCTION PROTOTYPES ================
void initUI();
void processUICommands();
void processUICommand(char* cmd);
void sendToUI(const char* message);
void sendToUILn(const char* message);
void sendToUIBinary(const uint8_t* data, size_t len);   // for binary file transfers

void handleFileManagementCommands(char* cmd, char* param);
void handleDiagCommands(char* cmd, char* param);
void handleSystemCommands(char* cmd);
void processConfigCommand(String cmd);
void sendSessionHistory();

void sendStatus();
void sendCardInfo();
void sendHelp();
void sendLiveData();
void sendStats();
void sendGPSInfo();
void applyConfigurationChanges();

#endif