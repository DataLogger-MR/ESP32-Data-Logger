#include "file_manager.h"
#include "sd_card.h"
#include "session_manager.h"
#include "ecu_state.h"
#include "data_logger.h"
#include "utils.h"
#include "config.h"
#include "miniz.h"
#include "globals.h"
#include "pins.h"
#include <time.h>
#include "driver/twai.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>

// ================ FILE MANAGER GLOBALS ================
char currentFilePath[128] = "";
unsigned long currentFileSize = 0;
RotateReason_t lastRotateReason = ROTATE_REASON_NONE;

// External references (defined in other modules)
extern uint32_t currentSessionId;
extern uint32_t currentFileSequence;
extern uint32_t sessionRecordCounter;
extern uint32_t fileRecordCounter;
extern ECUState_t ecuState;
extern SessionMetadata_t currentSession;
extern BattSt1_t battSt1;
extern CellVolt_t cellVolt;
extern McuMsg1_t mcuMsg1;
extern McuMsg2_t mcuMsg2;
extern twai_status_info_t twai_status;
extern bool busOffDetected;
extern unsigned long ecuDisconnectTimer;
extern unsigned long messageCount;
extern unsigned long acceptedCount;
extern bool sdReady;
extern bool recoveryMode;
extern uint32_t lastRecError;
extern uint32_t lastTecError;
extern HardwareSerial Serial2;
extern char sessionLogPath[];

// Add with other extern declarations
extern int canBaudRate;
extern int canRxQueueSize;
extern String mqttBroker;
extern int mqttPort;
extern String mqttTopic;
extern String mqttClientId;
extern String mqttUsername;
extern String mqttPassword;

// Forward declarations
void sendFileNormal(const char* fullPath, const char* displayName);
bool sendFileCompressed(const char* fullPath, const char* displayName);
void sendDataInChunks(uint8_t* data, size_t dataSize);
size_t simpleRLECompress(const uint8_t* input, size_t inputSize, uint8_t* output, size_t outputSize);

const char* getFileTypePrefix(FileType_t type) {
  switch(type) {
    case FILE_TYPE_DATA: return "DATA";
    case FILE_TYPE_DIAG_DISCONNECT: return "DIAG_DISCONNECT";
    case FILE_TYPE_DIAG_RECOVERY: return "DIAG_RECOVERY";
    case FILE_TYPE_DIAG_ERROR: return "DIAG_ERROR";
    case FILE_TYPE_DIAG_BUSOFF: return "DIAG_BUSOFF";
    case FILE_TYPE_SYSTEM: return "SYS";
    case FILE_TYPE_TEMP: return "TEMP";
    default: return "UNKNOWN";
  }
}

void initFileManager() {
  createDirectoryRecursive("/logs");
  createDirectoryRecursive("/system");
  createDirectoryRecursive("/temp");
  createDirectoryRecursive("/config");
  
}

void generateFilePath(char* buffer, size_t len, FileType_t fileType, 
                      uint32_t sessionId, uint32_t fileSeq, const char* suffix) {
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  
  const char* typePrefix = getFileTypePrefix(fileType);
  
  if (timeinfo->tm_year < 100) {
    // Time not yet set – place files directly in /logs/
    if (fileType == FILE_TYPE_DATA) {
      snprintf(buffer, len, "/logs/%s_%lu_S%uF%u.csv", 
               typePrefix, millis(), sessionId, fileSeq);
    } else {
      snprintf(buffer, len, "/logs/%s_%lu_%u%s.csv", 
               typePrefix, millis(), fileSeq, suffix ? suffix : "");
    }
    return;
  }
  
  // Time is set – use date-based subdirectories
  if (fileType == FILE_TYPE_DATA) {
    snprintf(buffer, len, "/logs/%04d/%02d/%02d/%s_%04d%02d%02d_%02d%02d_S%uF%u.csv",
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             typePrefix,
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             timeinfo->tm_hour, timeinfo->tm_min,
             sessionId, fileSeq);
  } else {
    snprintf(buffer, len, "/logs/%04d/%02d/%02d/%s_%04d%02d%02d_%02d%02d_%u%s.csv",
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             typePrefix,
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             timeinfo->tm_hour, timeinfo->tm_min,
             fileSeq, suffix ? suffix : "");
  }
}

void createNewLogFile() {
    unsigned long startTime = micros();   // DEBUG: timing

    if (!sdReady) return;

    // ========== CRITICAL CHANGE: Wait for time sync ==========
    // Check if system time is set (RTC, GPS, or NTP)
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    
    // If time is not set (year < 2020), don't create file yet
    if (timeinfo->tm_year < 120) {  // tm_year = years since 1900, so 120 = 2020
        Serial.println("⏳ Time not synchronized yet. Waiting for GPS/NTP time...");
        Serial2.println("TIME_SYNC_PENDING");
        
        // Don't create file, just return
        // The session will start automatically when time is synced and data arrives
        return;
    }
    // ==========================================================

    // *** NEW: Refresh the dynamic header if we are in dynamic mode ***
    if (dynamicMode) {
        updateDynamicHeader();   // ensures I2C signal names are up‑to‑date
    }

    currentFileSequence++;
    fileRecordCounter = 0;

    generateFilePath(currentFilePath, sizeof(currentFilePath),
                     FILE_TYPE_DATA, currentSessionId, currentFileSequence, "");

    char dirPath[128];
    strcpy(dirPath, currentFilePath);
    char* lastSlash = strrchr(dirPath, '/');
    if (lastSlash) {
        *lastSlash = '\0';
        if (!createDirectoryRecursive(dirPath)) {
            Serial.println("Failed to create log directory!");
            return;
        }
    }

    File logFile = SD.open(currentFilePath, FILE_WRITE);
    if (logFile) {
        logFile.printf("# SESSION_ID: %u, FILE_SEQUENCE: %u, START_TIME: %lu\n",
                       currentSessionId, currentFileSequence, millis() / 1000);

        // Write header based on mode
        if (dynamicMode) {
            logFile.println(getDynamicHeader().c_str());
        } else {
            logFile.println("Timestamp_ms,SessionRecord,FileRecord,SOC,Voltage_V,Current_A,MaxCellNo,MaxCell_mV,MinCellNo,MinCell_mV,MaxCellTemp,MaxCellTempNo,MinCellTemp,MinCellTempNo,AvgCellTemp,CapRemain_AH,FullCap_AH,CycleCap_AH,CycleCount,BMSRunTime_s,HeatCur_mA,SOH,CDCL_A,CCCL_A,PDCL_A,PCCL_A,ChgMos,DchgMos,Balance,Heater,ChargerPlug,ACC,Cell1_mV,Cell2_mV,Cell3_mV,Cell4_mV,Cell5_mV,Cell6_mV,Cell7_mV,Cell8_mV,Cell9_mV,Cell10_mV,Cell11_mV,Cell12_mV,Cell13_mV,Cell14_mV,Cell15_mV,Cell16_mV,MCU_DC_V,MCU_MotorTemp_C,MCU_CntrlTemp_C,MCU_Throttle,MCU_Speed_RPM,MCU_SpeedLimit_RPM,MCU_SpeedMode,AUX_Torque_Nm,AUX_Speed_RPM,AUX_CtrlTemp_C,AUX_MotorTemp_C,AUX_Cur_A,AUX_Volt_V,AUX_MCU_Volt_V,AUX_CanLife,ChgOut_Volt_V,ChgOut_Cur_A,ChgIn_AC_V,ChgIntTemp_C,Latitude,Longitude,Altitude_m,Speed_kmh,Speed_mps,Speed_knots,Course_deg,CardinalDir,Time_UTC,Date,Time_IST,Satellites,HDOP,Compass_deg,CompassDir,MagX_uT,MagY_uT,MagZ_uT,Uptime_s,MaxSpeed_kmh,TotalDist_km,RTC_Date,RTC_Time,RTC_Temp_C,TC1_Temp_C,TC1_Ambient_C,TC1_Fault,TC2_Temp_C,TC2_Ambient_C,TC2_Fault,TC3_Temp_C,TC3_Ambient_C,TC3_Fault,TC4_Temp_C,TC4_Ambient_C,TC4_Fault,TC5_Temp_C,TC5_Ambient_C,TC5_Fault,TC6_Temp_C,TC6_Ambient_C,TC6_Fault,TC7_Temp_C,TC7_Ambient_C,TC7_Fault,TC8_Temp_C,TC8_Ambient_C,TC8_Fault,TC9_Temp_C,TC9_Ambient_C,TC9_Fault,TC10_Temp_C,TC10_Ambient_C,TC10_Fault,TC11_Temp_C,TC11_Ambient_C,TC11_Fault,TC12_Temp_C,TC12_Ambient_C,TC12_Fault,GPIO_State,GPIO0,GPIO1,GPIO2,GPIO3,GPIO4,GPIO5,GPIO6,GPIO7,GPIO8,GPIO9,GPIO10,GPIO11,GPIO12,GPIO13,GPIO14,GPIO15,ADC0_V,ADC0_Raw,ADC1_V,ADC1_Raw,ADC2_V,ADC2_Raw,ADC3_V,ADC3_Raw");
        }

        logFile.close();

        currentFileSize = 0;
        Serial.printf("✅ SESSION %u FILE %u: %s\n", currentSessionId, currentFileSequence, currentFilePath);

        currentSession.sessionId = currentSessionId;
        currentSession.fileSequence = currentFileSequence;
        // startEpoch is already set when the session began (in startNewSession)
        strcpy(currentSession.fileName, currentFilePath);
        currentSession.sessionRecordCount = sessionRecordCounter;
        currentSession.fileRecordCount = 0;
        currentSession.ecuState = ecuState;
        currentSession.rotateReason = ROTATE_REASON_NONE;
        currentSession.fileSize = 0;
        currentSession.cleanClosure = false;

        Serial2.print("NEW_DATA_FILE:");
        Serial2.printf("S%uF%u:%s\n", currentSessionId, currentFileSequence, currentFilePath);
    } else {
        Serial.println("❌ Failed to create log file!");
    }

    unsigned long duration = micros() - startTime;   // DEBUG
    if (duration > 100000) {   // >100 ms
        Serial.printf("⚠️ createNewLogFile took %lu µs\n", duration);
    }
}

bool needsFileRotation() {
  if (!sdReady) return false;
  
  // Use runtime variable instead of compile-time constant
  if (rotateHourlyEnabled) {
    time_t now_time = time(nullptr);
    struct tm *timeinfo = localtime(&now_time);
    static int currentHour = -1;
    int newHour = timeinfo->tm_hour;
    
    if (timeinfo->tm_year < 100) {
      newHour = (millis() / 3600000) % 24;
    }
    
    if (currentHour != newHour && currentHour != -1) {
      currentHour = newHour;
      lastRotateReason = ROTATE_REASON_HOURLY;
      return true;
    }
    currentHour = newHour;
  }
  
  // Use runtime max file size
  if (currentFileSize >= (uint64_t)maxFileSizeMB * 1024 * 1024) {
    lastRotateReason = ROTATE_REASON_SIZE;
    return true;
  }
  
  return false;
}

void rotateFile(RotateReason_t reason) {
  unsigned long startTime = micros();   // DEBUG

  if (!sdReady) return;
  
  Serial.printf("Rotating file, reason: %d\n", reason);
  
  if (reason == ROTATE_REASON_ECU_DISCONNECT) {
    closeCurrentFile(reason);
    createDisconnectDiagFile(reason, lastRecError, lastTecError);
    return;
  }
  else if (reason == ROTATE_REASON_SIZE || reason == ROTATE_REASON_HOURLY || 
           reason == ROTATE_REASON_USER_COMMAND) {
    closeCurrentFile(reason);
    createNewLogFile();
    Serial.printf("Session %u continuing with file %u\n", 
                  currentSessionId, currentFileSequence);
  }
  else if (reason == ROTATE_REASON_SYSTEM) {
    closeCurrentFile(reason);
    createErrorDiagFile("SYSTEM", millis());
    createNewLogFile();
  }
  else if (reason == ROTATE_REASON_RECOVERY) {
    closeCurrentFile(reason);
    createRecoveryDiagFile();
    createNewLogFile();
  }

  unsigned long duration = micros() - startTime;   // DEBUG
  if (duration > 200000) {   // >200 ms
    Serial.printf("⚠️ rotateFile took %lu µs\n", duration);
  }
}

void closeCurrentFile(RotateReason_t reason) {
    if (!sdReady) return;

    // Set end epoch
    currentSession.endEpoch = time(nullptr);   // <-- NEW

    // --- Write header if sessions.csv does not exist ---
    if (!SD.exists(sessionLogPath)) {
        File headerFile = SD.open(sessionLogPath, FILE_WRITE);
        if (headerFile) {
            // Updated header with new column names
            headerFile.println("SessionID,FileSeq,StartTime,EndTime,FileName,SessionRecords,FileRecords,ECUState,RotateReason,FileSize_Bytes,CleanClosure");
            headerFile.close();
        }
    }
    // ----------------------------------------------------

    currentSession.sessionRecordCount = sessionRecordCounter;
    currentSession.fileRecordCount = fileRecordCounter;
    currentSession.ecuState = ecuState;
    currentSession.rotateReason = reason;
    currentSession.fileSize = currentFileSize;
    currentSession.cleanClosure = true;

    // Format timestamps for CSV
    String startStr = formatEpochToLocal(currentSession.startEpoch);
    String endStr = formatEpochToLocal(currentSession.endEpoch);

    // Add to session log
    File file = SD.open(sessionLogPath, FILE_APPEND);
    if (file) {
        file.printf("%u,%u,%s,%s,%s,%u,%u,%d,%d,%lu,%d\n",
                    currentSession.sessionId,
                    currentSession.fileSequence,
                    startStr.c_str(),
                    endStr.c_str(),
                    currentSession.fileName,
                    currentSession.sessionRecordCount,
                    currentSession.fileRecordCount,
                    currentSession.ecuState,
                    currentSession.rotateReason,
                    currentSession.fileSize,
                    currentSession.cleanClosure);
        file.close();
    }

    Serial.printf("Session %u, File %u closed: %u records, %lu bytes\n",
                  currentSession.sessionId, currentSession.fileSequence,
                  fileRecordCounter, currentFileSize);
}


void listFiles() {
  if (!sdReady) {
    Serial2.println("ERROR: SD not ready");
    return;
  }
  
  Serial2.println("FILE_LIST_BEGIN");
  
  File logsDir = SD.open("/logs");
  if (logsDir) {
    listFilesByType(logsDir, "DATA");
    logsDir.close();
  }
  
  Serial2.println("FILE_LIST_END");
  Serial.printf("✅ Listed DATA files\n");
}

void listDiagFiles() {
  if (!sdReady) {
    Serial2.println("ERROR: SD not ready");
    return;
  }
  
  Serial2.println("DIAG_LIST_BEGIN");
  
  File logsDir = SD.open("/logs");
  if (logsDir) {
    listFilesByType(logsDir, "DIAG");
    logsDir.close();
  }
  
  Serial2.println("DIAG_LIST_END");
  Serial.printf("✅ Listed DIAG files\n");
}

void listFilesByType(File dir, const char* prefix) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    
    if (entry.isDirectory()) {
      listFilesByType(entry, prefix);
    } else {
      String name = entry.name();
      if (name.startsWith(prefix)) {
        String fullPath = String(entry.name());
        int logsIndex = fullPath.indexOf("/logs/");
        if (logsIndex >= 0) {
          String relPath = fullPath.substring(logsIndex + 6);
          Serial2.print(relPath);
        } else {
          Serial2.print(name);
        }
        Serial2.print("|");
        Serial2.println(entry.size());
      }
    }
    entry.close();
  }
}

String findFileRecursive(const char* basePath, const char* targetFile) {
  File dir = SD.open(basePath);
  if (!dir) return "";
  
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    
    if (entry.isDirectory()) {
      String subPath = String(basePath) + "/" + String(entry.name());
      String result = findFileRecursive(subPath.c_str(), targetFile);
      if (result.length() > 0) {
        entry.close();
        dir.close();
        return result;
      }
    } else {
      String fileName = String(entry.name());
      int lastSlash = fileName.lastIndexOf('/');
      if (lastSlash >= 0) {
        fileName = fileName.substring(lastSlash + 1);
      }
      
      if (fileName == targetFile) {
        String fullPath = String(basePath) + "/" + String(entry.name());
        entry.close();
        dir.close();
        return fullPath;
      }
    }
    entry.close();
  }
  
  dir.close();
  return "";
}

void listDirContents(File dir, int level) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    
    for (int i = 0; i < level; i++) Serial.print("  ");
    Serial.print(entry.name());
    
    if (entry.isDirectory()) {
      Serial.println("/");
      listDirContents(entry, level + 1);
    } else {
      Serial.print("  Size: ");
      Serial.println(entry.size());
    }
    entry.close();
  }
}

void createDisconnectDiagFile(RotateReason_t reason, uint32_t lastRecErr, uint32_t lastTecErr) {
  if (!sdReady) return;
  
  static int diagCounter = 0;
  diagCounter++;
  
  char diagPath[128];
  char suffix[32];
  
  switch(reason) {
    case ROTATE_REASON_ECU_DISCONNECT: strcpy(suffix, "_DISCONNECT"); break;
    case ROTATE_REASON_SYSTEM: strcpy(suffix, "_SYSTEM"); break;
    case ROTATE_REASON_RECOVERY: strcpy(suffix, "_RECOVERY"); break;
    default: strcpy(suffix, "_EVENT");
  }
  
  generateFilePath(diagPath, sizeof(diagPath), 
                   FILE_TYPE_DIAG_DISCONNECT, 0, diagCounter, suffix);
  
  char dirPath[128];
  strcpy(dirPath, diagPath);
  char* lastSlash = strrchr(dirPath, '/');
  if (lastSlash) {
    *lastSlash = '\0';
    createDirectoryRecursive(dirPath);
  }
  
  File diagFile = SD.open(diagPath, FILE_WRITE);
  if (diagFile) {
    diagFile.println("=== ECU DISCONNECT DIAGNOSTIC REPORT ===");
    diagFile.printf("Timestamp: %lu ms\n", millis());
    diagFile.printf("Date: %s\n", __DATE__);
    diagFile.printf("Time: %s\n", __TIME__);
    diagFile.println();
    
    diagFile.println("=== DISCONNECT DETAILS ===");
    diagFile.printf("Reason: %d\n", reason);
    diagFile.printf("Last session: %u\n", currentSessionId);
    diagFile.printf("Last file: %u\n", currentFileSequence);
    diagFile.printf("Last session records: %u\n", sessionRecordCounter);
    diagFile.printf("Last file records: %u\n", fileRecordCounter);
    diagFile.printf("Disconnect duration: %lu ms\n", millis() - ecuDisconnectTimer);
    diagFile.println();
    
    diagFile.println("=== ERROR COUNTERS AT DISCONNECT ===");
    diagFile.printf("RX Error Counter: %lu\n", lastRecErr);
    diagFile.printf("TX Error Counter: %lu\n", lastTecErr);
    diagFile.printf("Bus Off Detected: %s\n", busOffDetected ? "YES" : "NO");
    diagFile.println();
    
    diagFile.println("=== LAST KNOWN GOOD DATA ===");
    diagFile.printf("Battery Voltage: %.2f V\n", battSt1.voltage);
    diagFile.printf("Battery Current: %.2f A\n", battSt1.current);
    diagFile.printf("State of Charge: %d %%\n", battSt1.soc);
    diagFile.printf("Max Cell Voltage: %u mV (Cell %d)\n", 
                    cellVolt.maxCellVolt, cellVolt.maxCellNo);
    diagFile.printf("Min Cell Voltage: %u mV (Cell %d)\n", 
                    cellVolt.minCellVolt, cellVolt.minCellNo);
    diagFile.printf("Motor Speed: %u RPM\n", mcuMsg2.motorSpeed);
    diagFile.printf("Controller Temp: %d °C\n", mcuMsg1.cntrlTemp);
    diagFile.println();
    
    diagFile.println("=== CAN BUS STATUS ===");
    diagFile.printf("State: %d\n", twai_status.state);
    diagFile.printf("RX Errors: %d\n", twai_status.rx_error_counter);
    diagFile.printf("TX Errors: %d\n", twai_status.tx_error_counter);
    diagFile.printf("TX Failed: %d\n", twai_status.tx_failed_count);
    diagFile.printf("RX Missed: %d\n", twai_status.rx_missed_count);
    diagFile.printf("RX Overrun: %d\n", twai_status.rx_overrun_count);
    diagFile.printf("Bus Error: %d\n", twai_status.bus_error_count);
    diagFile.println();
    
    diagFile.println("=== SYSTEM STATE ===");
    diagFile.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
    diagFile.printf("Uptime: %lu seconds\n", millis() / 1000);
    diagFile.printf("Total Messages: %lu\n", messageCount);
    diagFile.printf("Accepted Messages: %lu\n", acceptedCount);
    
    diagFile.close();
    
    Serial.printf("✅ DIAG FILE: %s\n", diagPath);
    Serial2.print("NEW_DIAG_FILE:");
    Serial2.println(diagPath);
  }
}

void createErrorDiagFile(const char* errorType, uint32_t errorCode) {
  if (!sdReady) return;
  
  static int errorCounter = 0;
  errorCounter++;
  
  char errorPath[128];
  char suffix[32];
  snprintf(suffix, sizeof(suffix), "_%s_%lu", errorType, errorCode);
  
  generateFilePath(errorPath, sizeof(errorPath), 
                   FILE_TYPE_DIAG_ERROR, 0, errorCounter, suffix);
  
  File errorFile = SD.open(errorPath, FILE_WRITE);
  if (errorFile) {
    errorFile.println("=== ERROR DIAGNOSTIC REPORT ===");
    errorFile.printf("Timestamp: %lu ms\n", millis());
    errorFile.printf("Error Type: %s\n", errorType);
    errorFile.printf("Error Code: %lu\n", errorCode);
    errorFile.close();
  }
}

void createRecoveryDiagFile() {
  if (!sdReady) return;
  
  static int recoveryCounter = 0;
  recoveryCounter++;
  
  char recoveryPath[128];
  generateFilePath(recoveryPath, sizeof(recoveryPath), 
                   FILE_TYPE_DIAG_RECOVERY, 0, recoveryCounter, "");
  
  File recFile = SD.open(recoveryPath, FILE_WRITE);
  if (recFile) {
    recFile.println("=== SYSTEM RECOVERY REPORT ===");
    recFile.printf("Timestamp: %lu ms\n", millis());
    recFile.printf("Recovery mode: %s\n", recoveryMode ? "ACTIVE" : "INACTIVE");
    recFile.printf("Last session before crash: %d\n", currentSessionId);
    recFile.close();
  }
}

void listDiagFilesRecursive(File dir, int level) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    
    if (entry.isDirectory()) {
      listDiagFilesRecursive(entry, level + 1);
    } else {
      String name = entry.name();
      if (name.startsWith("DIAG_")) {
        String fullPath = String(entry.name());
        int logsIndex = fullPath.indexOf("/logs/");
        if (logsIndex >= 0) {
          String relPath = fullPath.substring(logsIndex + 6);
          Serial2.print(relPath);
        } else {
          Serial2.print(entry.name());
        }
        Serial2.print("|");
        Serial2.print(entry.size());
        Serial2.print("|");
        Serial2.println(entry.getLastWrite());
      }
    }
    entry.close();
  }
}

// ================ FILE TRANSFER FUNCTIONS ================

void sendFile(const char* fileName) {
  if (!sdReady) {
    Serial2.println("ERROR: SD not ready");
    return;
  }
  
  String fullPath = findFileRecursive("/logs", fileName);
  
  if (fullPath.length() == 0) {
    if (fileName[0] == '/') {
      fullPath = String(fileName);
    } else {
      fullPath = "/logs/" + String(fileName);
    }
  }
  
  if (!SD.exists(fullPath)) {
    Serial2.println("ERROR: File not found");
    Serial.printf("File not found: %s\n", fullPath.c_str());
    return;
  }
  
  File file = SD.open(fullPath);
  size_t fileSize = file.size();
  file.close();
  
  Serial.printf("File: %s, Size: %d\n", fullPath.c_str(), fileSize);
  
  String displayName = fullPath;
  if (displayName.startsWith("/logs/")) {
    displayName = displayName.substring(6);
  }
  
  if (fileSize > 10 * 1024 && fileSize < 500 * 1024) {
    if (!sendFileCompressed(fullPath.c_str(), displayName.c_str())) {
      sendFileNormal(fullPath.c_str(), displayName.c_str());
    }
  } else {
    sendFileNormal(fullPath.c_str(), displayName.c_str());
  }
}

void sendFileNormal(const char* fullPath, const char* displayName) {
  File file = SD.open(fullPath);
  if (!file) {
    Serial2.println("ERROR: File not found");
    return;
  }
  
  size_t fileSize = file.size();
  
  Serial2.print("FILE_BEGIN:");
  Serial2.print(fileSize);
  Serial2.print(":");
  Serial2.println(displayName);
  
  uint8_t buffer[2048];
  size_t bytesRead;
  unsigned long startTime = millis();
  
  while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
    Serial2.write(buffer, bytesRead);
  }
  
  file.close();
  Serial2.println("FILE_END");
  
  unsigned long elapsed = millis() - startTime;
  float speed = (fileSize / 1024.0) / (elapsed / 1000.0);
  Serial.printf("✅ Normal: %d bytes in %lu ms (%.1f KB/s)\n", fileSize, elapsed, speed);
}

bool sendFileCompressed(const char* fullPath, const char* displayName) {
  File file = SD.open(fullPath);
  if (!file) {
    return false;
  }
  
  size_t fileSize = file.size();
  
  if (fileSize < 10240 || fileSize > 500 * 1024) {
    file.close();
    return false;
  }
  
  size_t freeHeap = ESP.getFreeHeap();
  Serial.printf("Free heap: %d bytes, File size: %d bytes\n", freeHeap, fileSize);
  
  if (fileSize > freeHeap * 0.7) {
    Serial.println("Not enough memory for compression");
    file.close();
    return false;
  }
  
  Serial.printf("Attempting to compress %d bytes...\n", fileSize);
  
  uint8_t* fileData = (uint8_t*)malloc(fileSize);
  if (!fileData) {
    Serial.println("Failed to allocate memory for file");
    file.close();
    return false;
  }
  
  file.read(fileData, fileSize);
  file.close();
  
  size_t maxCompressedSize = fileSize + (fileSize / 2) + 128;
  uint8_t* compressedData = (uint8_t*)malloc(maxCompressedSize);
  if (!compressedData) {
    free(fileData);
    Serial.println("Failed to allocate compression buffer");
    return false;
  }
  
  size_t compressedSize = simpleRLECompress(fileData, fileSize, compressedData, maxCompressedSize);
  
  if (compressedSize > 0 && compressedSize < fileSize) {
    float ratio = (100.0 * compressedSize) / fileSize;
    Serial.printf("RLE compressed: %d → %d bytes (%.1f%%)\n", fileSize, compressedSize, ratio);
    
    Serial2.print("FILE_RLE:");
    Serial2.print(fileSize);
    Serial2.print(":");
    Serial2.print(compressedSize);
    Serial2.print(":");
    Serial2.println(displayName);
    
    sendDataInChunks(compressedData, compressedSize);
    
    free(fileData);
    free(compressedData);
    return true;
  }
  
  free(fileData);
  free(compressedData);
  return false;
}

void sendDataInChunks(uint8_t* data, size_t dataSize) {
  uint8_t* ptr = data;
  size_t remaining = dataSize;
  size_t chunkSize = 2048;
  unsigned long startTime = millis();
  
  while (remaining > 0) {
    size_t sendSize = (remaining > chunkSize) ? chunkSize : remaining;
    Serial2.write(ptr, sendSize);
    ptr += sendSize;
    remaining -= sendSize;
  }
  
  Serial2.println("FILE_END");
  
  unsigned long elapsed = millis() - startTime;
  float speed = (dataSize / 1024.0) / (elapsed / 1000.0);
  Serial.printf("✅ Sent %d bytes in %lu ms (%.1f KB/s)\n", dataSize, elapsed, speed);
}

size_t simpleRLECompress(const uint8_t* input, size_t inputSize, uint8_t* output, size_t outputSize) {
  size_t outPos = 0;
  size_t inPos = 0;
  
  while (inPos < inputSize && outPos < outputSize - 3) {
    uint8_t current = input[inPos];
    size_t runLength = 1;
    
    while (inPos + runLength < inputSize && 
           input[inPos + runLength] == current && 
           runLength < 255) {
      runLength++;
    }
    
    if (runLength > 3 || current == 0xFC) {
      if (outPos + 3 <= outputSize) {
        output[outPos++] = 0xFC;
        output[outPos++] = runLength;
        output[outPos++] = current;
      }
    } else {
      for (size_t i = 0; i < runLength; i++) {
        if (outPos < outputSize) {
          output[outPos++] = current;
        }
      }
    }
    
    inPos += runLength;
  }
  
  return outPos;
}

// ================ UTILITY FUNCTIONS ================

void createTestFile(const char* fileName) {
  if (!sdReady) {
    Serial2.println("ERROR: SD not ready");
    return;
  }
  
  String fullPath = fileName;
  if (fileName[0] != '/') fullPath = "/" + String(fileName);
  
  // Ensure we're creating in logs directory
  if (!fullPath.startsWith("/logs/")) {
    fullPath = "/logs" + fullPath;
  }
  
  // Create directory if needed
  char dirPath[128];
  strcpy(dirPath, fullPath.c_str());
  char* lastSlash = strrchr(dirPath, '/');
  if (lastSlash) {
    *lastSlash = '\0';
    createDirectoryRecursive(dirPath);
  }
  
  File testFile = SD.open(fullPath, FILE_WRITE);
  if (!testFile) {
    Serial2.println("ERROR: Cannot create file");
    Serial.printf("Failed to create: %s\n", fullPath.c_str());
    return;
  }
  
  testFile.println("Timestamp,SessionRecord,FileRecord,Value1,Value2");
  for (int i = 0; i < 10; i++) {
    testFile.printf("%lu,%d,%d,%d,%d\n", millis(), i, i, i, i * 2);
  }
  
  testFile.close();
  Serial2.printf("OK: Created %s\n", fullPath.c_str());
  Serial.println("✅ Test file created");
}

void deleteFile(const char* fileName) {
  if (!sdReady) {
    Serial2.println("ERROR: SD not ready");
    return;
  }
  
  String fullPath = findFileRecursive("/logs", fileName);
  
  if (fullPath.length() == 0) {
    if (fileName[0] == '/') {
      fullPath = String(fileName);
    } else {
      fullPath = "/logs/" + String(fileName);
    }
  }
  
  if (SD.remove(fullPath)) {
    Serial2.print("OK: Deleted ");
    Serial2.println(fullPath);
    Serial.println("✅ File deleted");
  } else {
    Serial2.println("ERROR: Delete failed");
    Serial.printf("Failed to delete: %s\n", fullPath.c_str());
  }
}

void listLogsByDate(int year, int month, int day) {
  if (!sdReady) {
    Serial2.println("ERROR: SD not ready");
    return;
  }
  
  char path[64];
  snprintf(path, sizeof(path), "/logs/%04d/%02d/%02d", year, month, day);
  
  if (!SD.exists(path)) {
    Serial2.println("No logs for this date");
    return;
  }
  
  File dir = SD.open(path);
  if (!dir) {
    Serial2.println("ERROR: Cannot open directory");
    return;
  }
  
  Serial2.println("LOG_LIST_BEGIN");
  
  File file = dir.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      Serial2.print(file.name());
      Serial2.print("|");
      Serial2.print(file.size());
      Serial2.print("|");
      Serial2.println(file.getLastWrite());
    }
    file.close();
    file = dir.openNextFile();
  }
  dir.close();
  
  Serial2.println("LOG_LIST_END");
}

bool saveConfigFile(const char* path, const uint8_t* data, size_t len) {
    File file = SD.open(path, FILE_WRITE);
    if (!file) return false;
    file.write(data, len);
    file.close();
    return true;
}

bool loadConfigFile(const char* path, String& content) {
    File file = SD.open(path, FILE_READ);
    if (!file) return false;
    content = file.readString();
    file.close();
    return true;
}

bool deleteConfigFile(const char* path) {
    return SD.remove(path);
}

void saveConfigToSPIFFS() {
    Serial.println("📝 saveConfigToSPIFFS() called");
    
    DynamicJsonDocument doc(2048);
    
    doc["logging"]["interval_ms"] = logIntervalMs;
    doc["logging"]["max_file_size_mb"] = maxFileSizeMB;
    doc["logging"]["rotate_hourly"] = rotateHourlyEnabled;
    doc["logging"]["auto_delete_days"] = autoDeleteDays;
    
    doc["gps"]["baud_rate"] = gpsBaudRate;
    doc["gps"]["update_interval_ms"] = gpsUpdateInterval;
    
    doc["wifi"]["ssid"] = wifiSSID;
    doc["wifi"]["password"] = wifiPassword;
    
    doc["system"]["buffer_size"] = bufferSize;
    doc["system"]["ecu_timeout"] = ecuTimeout;
    
    // Add CAN settings
    doc["can"]["baud_rate"] = canBaudRate;
    doc["can"]["rx_queue_size"] = canRxQueueSize;
    
    // Add MQTT settings
    doc["mqtt"]["broker"] = mqttBroker;
    doc["mqtt"]["port"] = mqttPort;
    doc["mqtt"]["topic"] = mqttTopic;
    doc["mqtt"]["client_id"] = mqttClientId;
    doc["mqtt"]["username"] = mqttUsername;
    doc["mqtt"]["password"] = mqttPassword;
    
    // Make sure the /config directory exists
    if (!SPIFFS.exists("/config")) {
        if (SPIFFS.mkdir("/config")) {
            Serial.println("✅ Created /config directory");
        } else {
            Serial.println("❌ Failed to create /config directory");
        }
    }
    
    // Also check if SPIFFS is mounted
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ SPIFFS not mounted!");
        return;
    }
    
    File file = SPIFFS.open("/config/settings.json", FILE_WRITE);
    if (file) {
        size_t bytesWritten = serializeJson(doc, file);
        if (bytesWritten > 0) {
            Serial.printf("✅ Configuration saved to SPIFFS (%d bytes)\n", bytesWritten);
            Serial.printf("   Saved values: interval=%d, maxSize=%d, gpsBaud=%d\n", 
                         logIntervalMs, maxFileSizeMB, gpsBaudRate);
            
            // Verify by reading back
            file.close();
            File verifyFile = SPIFFS.open("/config/settings.json", FILE_READ);
            if (verifyFile) {
                String content = verifyFile.readString();
                Serial.printf("   Verification: Read %d bytes, first 100 chars: %s\n", 
                             content.length(), content.substring(0, 100).c_str());
                verifyFile.close();
            } else {
                Serial.println("❌ Verification: Cannot read saved file!");
            }
        } else {
            Serial.println("❌ Failed to write configuration to SPIFFS");
        }
        file.close();
    } else {
        Serial.println("❌ Failed to open config file for writing");
        
        // List SPIFFS contents for debugging
        Serial.println("SPIFFS contents:");
        File root = SPIFFS.open("/");
        File f = root.openNextFile();
        while (f) {
            Serial.printf("  %s (%d bytes)\n", f.name(), f.size());
            f = root.openNextFile();
        }
        root.close();
    }
}

void loadConfigFromSPIFFS() {
    // Check if SPIFFS is mounted
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ SPIFFS Mount Failed");
        return;
    }
    
    if (!SPIFFS.exists("/config/settings.json")) {
        Serial.println("No saved config, using defaults");
        return;
    }
    
    File file = SPIFFS.open("/config/settings.json", FILE_READ);
    if (!file) {
        Serial.println("Failed to open config file");
        return;
    }
    
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.println("Failed to parse config file");
        return;
    }
    
    // Read values
    logIntervalMs = doc["logging"]["interval_ms"] | 100;
    maxFileSizeMB = doc["logging"]["max_file_size_mb"] | 100;
    rotateHourlyEnabled = doc["logging"]["rotate_hourly"] | false;
    autoDeleteDays = doc["logging"]["auto_delete_days"] | 30;
    
    gpsBaudRate = doc["gps"]["baud_rate"] | 38400;
    gpsUpdateInterval = doc["gps"]["update_interval_ms"] | 100;
    
    wifiSSID = doc["wifi"]["ssid"] | "";
    wifiPassword = doc["wifi"]["password"] | "";
    
    bufferSize = doc["system"]["buffer_size"] | 16384;
    ecuTimeout = doc["system"]["ecu_timeout"] | 30000;
    
    // CAN Settings - use defaults directly (no #defines)
    canBaudRate = doc["can"]["baud_rate"] | 500;
    canRxQueueSize = doc["can"]["rx_queue_size"] | 100;
    
    // MQTT Settings - use defaults directly (no #defines from wifi_manager.h)
    mqttBroker = doc["mqtt"]["broker"] | "01792b66dfee4540a546dc894922fb94.s1.eu.hivemq.cloud";
    mqttPort = doc["mqtt"]["port"] | 8883;
    mqttTopic = doc["mqtt"]["topic"] | "tractor/data";
    mqttClientId = doc["mqtt"]["client_id"] | "ESP32_Tractor_Logger";
    mqttUsername = doc["mqtt"]["username"] | "MR_TRACTOR";
    mqttPassword = doc["mqtt"]["password"] | "#Lokesh000";
    
    currentGpsBaud = gpsBaudRate;
    currentBufferSize = bufferSize;
    
}