#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "types.h"
#include <set> 
#include <map>

#include "speed_sensor.h"  

// ================ GLOBAL VARIABLES DECLARATIONS ================
extern bool sdReady;
extern bool loggingActive;
extern unsigned long messageCount;
extern unsigned long acceptedCount;
extern unsigned long filteredOutCount;
extern unsigned long loggedCount;
extern unsigned long startTime;
extern unsigned long lastStatsTime;
extern unsigned long lastFlushTime;
extern unsigned long lastLogTime;
extern char sessionLogPath[];

extern uint32_t currentSessionId;
extern uint32_t currentFileSequence;
extern uint32_t sessionRecordCounter;
extern uint32_t fileRecordCounter;
extern SessionState_t sessionState;
extern SessionMetadata_t currentSession;
extern bool firstConnection;
extern bool recoveryMode;

extern ECUState_t ecuState;
extern unsigned long ecuDisconnectTimer;
extern unsigned long ecuLastMessageTime;
extern int ecuDisconnectSampleCount;
extern int ecuConnectSampleCount;
extern uint32_t lastRecError;
extern uint32_t lastTecError;
extern uint32_t recErrorAccumulator;
extern uint32_t tecErrorAccumulator;
extern uint32_t errorSampleCount;
extern bool busOffDetected;

extern char currentFilePath[128];
extern unsigned long currentFileSize;
extern RotateReason_t lastRotateReason;

extern BattSt1_t battSt1;
extern CellVolt_t cellVolt;
extern CellTemp_t cellTemp;
extern AlmInfo_t almInfo;
extern Bms6_t bms6;
extern BattSt2_t battSt2;
extern AllTemp_t allTemp;
extern BmsErrInfo_t bmsErrInfo;
extern BmsInfo_t bmsInfo;
extern BmsSwSta_t bmsSwSta;
extern CellVoltages_t cellVoltages;
extern BmsChgInfo_t bmsChgInfo;
extern CtrlInfo_t ctrlInfo;
extern McuMsg1_t mcuMsg1;
extern McuMsg2_t mcuMsg2;
extern McuMsg3_t mcuMsg3;
extern McuMsg4_t mcuMsg4;
extern McuMsg5_t mcuMsg5;
extern McuMsg6_t mcuMsg6;
extern AuxMotor1_t auxMotor1;
extern AuxMotor2_t auxMotor2;
extern AuxMotor3_t auxMotor3;
extern ChrgOut_t chrgOut;
extern SemaphoreHandle_t dataMutex;   
extern bool uartDataPresent;
extern std::set<String> selectedUartSignals;

extern String wifiStatus;
extern String wifiIP;

extern bool dynamicMode;
extern unsigned long lastCANActivity;
extern unsigned long lastUARTActivity;
extern bool dataActive;  

extern unsigned long filteredMessageCount;
extern unsigned long lastFilteredTime;
extern std::map<String, double> i2cValues;


// ================ RUNTIME CONFIGURATION VARIABLES ================
extern int logIntervalMs;
extern int maxFileSizeMB;
extern bool rotateHourlyEnabled;
extern int autoDeleteDays;
extern int gpsBaudRate;
extern int gpsUpdateInterval;
extern String wifiSSID;
extern String wifiPassword;
extern int bufferSize;
extern int ecuTimeout;
extern int csvLineBufferSize;
extern int currentGpsBaud;
extern int currentBufferSize;

extern int canBaudRate;
extern int canRxQueueSize;

extern String mqttBroker;
extern int mqttPort;
extern String mqttTopic;
extern String mqttClientId;
extern String mqttUsername;
extern String mqttPassword;

#endif