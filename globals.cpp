#include "globals.h"
#include "types.h"
#include <set>   

// ================ GLOBAL VARIABLES DEFINITIONS ================
bool loggingActive = true;
unsigned long lastStatsTime = 0;
unsigned long lastFlushTime = 0;
unsigned long lastLogTime = 0;
char sessionLogPath[] = "/system/sessions.csv";
bool dynamicMode = false;
bool uartDataPresent = false;  // GPS uses this
std::set<String> selectedUartSignals;  // Can be removed if not used elsewhere
unsigned long lastCANActivity = 0;
unsigned long lastUARTActivity = 0;  // GPS uses this
bool dataActive = false;
unsigned long lastFilteredTime = 0;
unsigned long filteredMessageCount = 0;
// NEW: I2C values map
std::map<String, double> i2cValues;

// Runtime configuration variables
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

// CAN Settings
int canBaudRate = 500;
int canRxQueueSize = 100;

// MQTT Settings
String mqttBroker = "01792b66dfee4540a546dc894922fb94.s1.eu.hivemq.cloud";
int mqttPort = 8883;
String mqttTopic = "tractor/data";
String mqttClientId = "ESP32_Tractor_Logger";
String mqttUsername = "MR_TRACTOR";
String mqttPassword = "#Lokesh000";