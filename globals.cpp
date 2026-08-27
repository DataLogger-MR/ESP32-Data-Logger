#include "globals.h"
#include "types.h"
#include <set>   
#include "speed_sensor.h"

// ================ GLOBAL VARIABLES DEFINITIONS ================
bool loggingActive = true;
unsigned long lastStatsTime = 0;
unsigned long lastFlushTime = 0;
unsigned long lastLogTime = 0;
char sessionLogPath[] = "/system/sessions.csv";
bool dynamicMode = false;
bool uartDataPresent = false;  
std::set<String> selectedUartSignals;  
unsigned long lastCANActivity = 0;
unsigned long lastUARTActivity = 0;  
bool dataActive = false;
unsigned long lastFilteredTime = 0;
unsigned long filteredMessageCount = 0;

SpeedData speedData = {0, 0, false, 1000};
AuxSpeedData auxSpeedData = {0, 0, false, 1000};

std::map<String, double> i2cValues;

TorqueData_t g_torque = {0};
SemaphoreHandle_t torqueMutex = NULL;

SemaphoreHandle_t i2cValuesMutex = NULL;
SemaphoreHandle_t gpsMutex = NULL;

int logIntervalMs = 100;
int maxFileSizeMB = 100;
bool rotateHourlyEnabled = false;
int autoDeleteDays = 30;
int gpsBaudRate = 38400;
int gpsUpdateInterval = 100;
String wifiSSID = "";
String wifiPassword = "";
int bufferSize = 16384;
int ecuTimeout = 30000;
int csvLineBufferSize = 2048;
int currentGpsBaud = 38400;
int currentBufferSize = 16384;

int canBaudRate = 500;
int canRxQueueSize = 100;

String mqttBroker = "01792b66dfee4540a546dc894922fb94.s1.eu.hivemq.cloud";
int mqttPort = 8883;
String mqttTopic = "tractor/data";
String mqttClientId = "ESP32_Tractor_Logger";
String mqttUsername = "MR_TRACTOR";
String mqttPassword = "#Lokesh000";

// ================ UI MODE STATE ================
bool uiModeActive = false;
unsigned long lastUICommandTime = 0;
SemaphoreHandle_t uiModeMutex = NULL;

// ================ UI MODE HELPERS ================

bool isUIActive() {
    if (uiModeMutex == NULL) return false;
    bool active = false;
    if (xSemaphoreTake(uiModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Forced ON always active
        if (uiModeActive) {
            active = true;
        } else {
            // No command ever received → not active
            if (lastUICommandTime == 0) {
                active = false;
            } else {
                active = (millis() - lastUICommandTime < UI_TIMEOUT_MS);
            }
        }
        xSemaphoreGive(uiModeMutex);
    }
    return active;
}

void setUIMode(bool active) {
    if (uiModeMutex == NULL) return;
    if (xSemaphoreTake(uiModeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uiModeActive = active;
        if (active) {
            // When forcing ON, update timestamp so it doesn't time out immediately
            lastUICommandTime = millis();
        }
        xSemaphoreGive(uiModeMutex);
    }
}