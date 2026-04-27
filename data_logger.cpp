#include "data_logger.h"
#include "sd_card.h"
#include "session_manager.h"
#include "ecu_state.h"
#include "can_decoder.h"
#include "file_manager.h"
#include "utils.h"
#include "globals.h"
#include "pins.h"
#include "gps_globals.h"
#include <map>
#include "dynamic_decoder.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "gps_globals.h"
#include "i2c_sensors.h"
#include "i2c_config.h"          

// ================ DOUBLE BUFFER GLOBALS ================
char* buffers[2] = {nullptr, nullptr};
int activeBufferIndex = 0;
int bufferIndex = 0;
SemaphoreHandle_t bufferMutex = nullptr;
QueueHandle_t flushQueue = nullptr;
SemaphoreHandle_t bufferFreeSem[2] = {nullptr, nullptr};

typedef struct {
    int bufferIdx;
    int dataSize;
} FlushItem_t;

// ================ DATA LOGGER GLOBALS ================
unsigned long loggedCount = 0;
unsigned long messageCount = 0;
unsigned long acceptedCount = 0;
unsigned long filteredOutCount = 0;
unsigned long startTime = 0;

String dynamicHeader = "";
std::map<String, double> lastDynamicValues;

// ================ I2C dynamic values ================
extern std::map<String, double> i2cValues;   

// ================ DEBUGGING GLOBALS ================
static unsigned long logCallCount = 0;
static unsigned long lastLogCallPrint = 0;

// ================ FLUSH TASK SYNC ================
extern SemaphoreHandle_t flushSemaphore;   

extern unsigned long lastFlushTime;
extern unsigned long lastLogTime;
extern char currentFilePath[128];
extern unsigned long currentFileSize;
extern uint32_t sessionRecordCounter;
extern uint32_t fileRecordCounter;
extern SessionState_t sessionState;
extern ECUState_t ecuState;
extern bool sdReady;
extern RotateReason_t lastRotateReason;

extern bool needsFileRotation();
extern void rotateFile(RotateReason_t reason);
extern void createNewLogFile();
extern void incrementSessionRecords();
extern void incrementFileRecords();

extern BattSt1_t battSt1;
extern CellVolt_t cellVolt;
extern CellTemp_t cellTemp;
extern BattSt2_t battSt2;
extern BmsInfo_t bmsInfo;
extern Bms6_t bms6;
extern BmsSwSta_t bmsSwSta;
extern CellVoltages_t cellVoltages;
extern McuMsg1_t mcuMsg1;
extern McuMsg2_t mcuMsg2;
extern AuxMotor1_t auxMotor1;
extern AuxMotor2_t auxMotor2;
extern ChrgOut_t chrgOut;
extern bool dynamicMode;

// ================ INITIALIZATION ================
void initDataLogger() {
   
    if (buffers[0] == nullptr) {
        buffers[0] = (char*) malloc(bufferSize);
        buffers[1] = (char*) malloc(bufferSize);

        bufferMutex = xSemaphoreCreateMutex();
        flushQueue = xQueueCreate(2, sizeof(FlushItem_t));   
        bufferFreeSem[0] = xSemaphoreCreateBinary();
        bufferFreeSem[1] = xSemaphoreCreateBinary();
        xSemaphoreGive(bufferFreeSem[0]);   
        xSemaphoreGive(bufferFreeSem[1]);
    }

    activeBufferIndex = 0;
    bufferIndex = 0;
    loggedCount = 0;
    messageCount = 0;
    acceptedCount = 0;
    filteredOutCount = 0;
    startTime = millis();

}

// ================ ADD TO BUFFER (non‑blocking) ================
void addToBuffer(const char* data, int len) {
    if (!buffers[0] || !buffers[1]) return;   
    if (len <= 0) return;

    xSemaphoreTake(bufferMutex, portMAX_DELAY);

    if (bufferIndex + len >= bufferSize - 1) {
      
        int oldBuffer = activeBufferIndex;
        int newBuffer = (activeBufferIndex + 1) % 2;

        if (xSemaphoreTake(bufferFreeSem[newBuffer], pdMS_TO_TICKS(1000)) != pdTRUE) {
           
            Serial.println("⚠️ addToBuffer: waiting for free buffer...");
            xSemaphoreTake(bufferFreeSem[newBuffer], portMAX_DELAY);
        }

        FlushItem_t item = {oldBuffer, bufferIndex};
        xQueueSend(flushQueue, &item, 0);

        activeBufferIndex = newBuffer;
        bufferIndex = 0;

    }

    memcpy(buffers[activeBufferIndex] + bufferIndex, data, len);
    bufferIndex += len;

    xSemaphoreGive(bufferMutex);
}

// ================ FLUSH TASK (to be called from FreeRTOS task) ================
void flushBufferTask(void *pvParameters) {
    FlushItem_t item;
    const TickType_t maxWait = pdMS_TO_TICKS(2000);   
    while (1) {
        
        if (xQueueReceive(flushQueue, &item, maxWait) == pdTRUE) {
            
            if (item.bufferIdx < 0 || item.bufferIdx > 1) continue;   
            if (item.dataSize <= 0) {
               
                xSemaphoreGive(bufferFreeSem[item.bufferIdx]);
                continue;
            }

            if (sdReady && buffers[item.bufferIdx]) {
                
                if (needsFileRotation()) {
                    rotateFile(lastRotateReason);
                }

                File logFile = SD.open(currentFilePath, FILE_APPEND);
                if (logFile) {
                    size_t written = logFile.write((uint8_t*)buffers[item.bufferIdx], item.dataSize);
                    logFile.close();
                    currentFileSize += written;
                    digitalWrite(LED_PIN, HIGH);
                    delay(5);
                    digitalWrite(LED_PIN, LOW);
                } else {
                    Serial.println("❌ Failed to open file for writing!");
                    if (!SD.exists(currentFilePath)) {
                        createNewLogFile();
                    }
                }
            }

            xSemaphoreGive(bufferFreeSem[item.bufferIdx]);
        } else {
           
        }
    }
}

// ================ LEGACY FUNCTIONS (modified to use new system) ================
void flushBuffer() {
}

// ================ FORMAT A COMPLETE CSV LINE ================
void formatCompleteCSV(char* buffer, size_t bufferSize, 
                       unsigned long timestampMs, unsigned long currentTime) {
    int pos = 0;
    
    String ts = getFormattedTime();
    pos += snprintf(buffer + pos, bufferSize - pos, "%s,%u,%u",
                    ts.c_str(), sessionRecordCounter, fileRecordCounter);
    
    if (isValid(battSt1.lastUpdate, battSt1.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%.3f,%.3f", 
                        battSt1.soc, battSt1.voltage, battSt1.current);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    if (isValid(cellVolt.lastUpdate, cellVolt.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%u,%d,%u",
                        cellVolt.maxCellNo, cellVolt.maxCellVolt,
                        cellVolt.minCellNo, cellVolt.minCellVolt);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    if (isValid(cellTemp.lastUpdate, cellTemp.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%d,%d,%d,%d",
                        cellTemp.maxCellTemp, cellTemp.maxCtNO,
                        cellTemp.minCellTemp, cellTemp.minCtNO,
                        cellTemp.avgCellTemp);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,");
    }
    
    if (isValid(battSt2.lastUpdate, battSt2.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%u",
                        battSt2.capRemain, battSt2.fulChargeCap,
                        battSt2.cycleCap, battSt2.cycleCount);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    if (isValid(bmsInfo.lastUpdate, bmsInfo.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%lu,%u,%d",
                        bmsInfo.bmsRunTime, bmsInfo.heatCur, bmsInfo.soh);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    if (isValid(bms6.lastUpdate, bms6.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%.1f",
                        bms6.cdcl, bms6.cccl, bms6.pdcl, bms6.pccl);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    if (isValid(bmsSwSta.lastUpdate, bmsSwSta.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%d,%d,%d,%d,%d",
                        bmsSwSta.chgMosSta, bmsSwSta.dchgMosSta,
                        bmsSwSta.balanSta, bmsSwSta.heatSta,
                        bmsSwSta.chgDevPlugSta, bmsSwSta.accSta);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,");
    }
    
    if (isValid(cellVoltages.lastUpdate, cellVoltages.timeoutMs, currentTime)) {
        for (int i = 0; i < 16; i++) {
            if (i < cellVoltages.cellCount) {
                pos += snprintf(buffer + pos, bufferSize - pos, ",%u", cellVoltages.cellVoltages[i]);
            } else {
                pos += snprintf(buffer + pos, bufferSize - pos, ",");
            }
        }
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,,,,,,,,,,,,");
    }
    
    if (isValid(mcuMsg1.lastUpdate, mcuMsg1.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%d,%d,%d",
                        mcuMsg1.dcVolt, mcuMsg1.motorTemp,
                        mcuMsg1.cntrlTemp, mcuMsg1.throttlePercent);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    if (isValid(mcuMsg2.lastUpdate, mcuMsg2.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%u,%u,%u",
                        mcuMsg2.motorSpeed, mcuMsg2.motorSpdLim, mcuMsg2.speedMode);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    if (isValid(auxMotor1.lastUpdate, auxMotor1.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%u,%d,%d",
                        auxMotor1.torqueEst, auxMotor1.speedEst,
                        auxMotor1.controllerTemp, auxMotor1.motorTemp);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    if (isValid(auxMotor2.lastUpdate, auxMotor2.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%u",
                        auxMotor2.motorCurrent, auxMotor2.motorVoltage,
                        auxMotor2.mcuVoltage, auxMotor2.canLife);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    if (isValid(chrgOut.lastUpdate, chrgOut.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%d",
                        chrgOut.chargerVoltageOut, chrgOut.chargerCurrentOut,
                        chrgOut.chargerInputAcVolt, chrgOut.chargerInternalTemp);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // ================ GPS DATA SECTION ================
    static unsigned long lastGpsDebug = 0;
    if (currentTime - lastGpsDebug > 10000) {
        Serial.printf("GPS Debug - Initialized: %d, Valid: loc=%d, time=%d, speed=%d, sat=%d\n",
                     gpsInitialized, gpsData.location_valid, gpsData.time_valid, 
                     gpsData.speed_valid, gpsData.satellites_valid);
        if (gpsData.location_valid) {
            Serial.printf("  Position: %.6f, %.6f\n", gpsData.latitude, gpsData.longitude);
        }
        if (gpsData.time_valid) {
            Serial.printf("  UTC Time: %02d:%02d:%02d\n", 
                         gpsData.hour_utc, gpsData.minute_utc, gpsData.second_utc);
        }
        lastGpsDebug = currentTime;
    }
    
    if (gpsData.location_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.6f,%.6f", 
                        gpsData.latitude, gpsData.longitude);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,");
    }
    
    if (gpsData.altitude_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f", gpsData.altitude);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (gpsData.speed_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f", 
                        gpsData.speed_kmh, gpsData.speed_mps, gpsData.speed_knots);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    if (gpsData.course_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%s", 
                        gpsData.course_deg, gpsData.cardinal_direction.c_str());
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,");
    }
    
    if (gpsData.time_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%02d:%02d:%02d",
                        gpsData.hour_utc, gpsData.minute_utc, gpsData.second_utc);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (gpsData.date_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%04d-%02d-%02d",
                        gpsData.year, gpsData.month, gpsData.day);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (gpsData.time_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%02d:%02d:%02d",
                        gpsData.hour_ist, gpsData.minute_ist, gpsData.second_utc);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (gpsData.satellites_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d", gpsData.satellites);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (gpsData.hdop_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f", gpsData.hdop);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (compassData.valid && isValid(compassData.lastUpdate, 5000, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%s,%.1f,%.1f,%.1f",
                        compassData.heading_deg, compassData.cardinal_direction.c_str(),
                        compassData.raw_x, compassData.raw_y, compassData.raw_z);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,");
    }
    
    unsigned long uptime_seconds = (currentTime - gpsStats.uptime_ms) / 1000;
    pos += snprintf(buffer + pos, bufferSize - pos, ",%lu,%.1f,%.3f",
                    uptime_seconds, gpsStats.maxSpeed_kmh, gpsStats.totalDistance_km);


// ================ SPEED SENSOR SECTION ================

pos += snprintf(buffer + pos, bufferSize - pos, ",%.0f", speedData.rpm);
      
// ================ I2C SENSORS DATA SECTION (static mode) ================
    formatI2CData(buffer + pos, bufferSize - pos, currentTime);
    
    snprintf(buffer + pos, bufferSize - pos, "\n");
}

// ================ LOG DATA TO SD (called every 100ms) ================
void logDataToSD() {
    logCallCount++;   

    if (!sdReady) return;
    if (sessionState != SESSION_STATE_ACTIVE) return;

    if (strlen(currentFilePath) == 0) {  
        createNewLogFile();
        if (strlen(currentFilePath) == 0) {
            Serial.println("Failed to create log file!");
            return;
        }
    }

    if (dynamicMode) {
        bool canDataAvailable = (ecuState == ECU_STATE_CONNECTED || ecuState == ECU_STATE_DEGRADED);
        if (!canDataAvailable && !uartDataPresent) {
            return;
        }
        logDynamicDataToSD();
    } else {
        unsigned long now = millis();
        unsigned long elapsed_ms = now - startTime;

        incrementSessionRecords();
        incrementFileRecords();

        char line[csvLineBufferSize];
        formatCompleteCSV(line, sizeof(line), elapsed_ms, now);

        if (strlen(line) > 20) {
            addToBuffer(line, strlen(line));
            loggedCount++;
            if (PRINT_LOGGED_DATA) {
                Serial.print("📝 LOG: ");
                Serial.print(line);
            }
        }
    }

    if (millis() - lastLogCallPrint > 5000) {
        Serial.printf("logDataToSD() calls/sec: %.2f\n", logCallCount / 5.0);
        lastLogCallPrint = millis();
        logCallCount = 0;
    }
}

// ================ RESET STATISTICS ================
void resetStatistics() {
    messageCount = 0;
    acceptedCount = 0;
    filteredOutCount = 0;
    loggedCount = 0;
    startTime = millis();
    Serial2.println("STATS_RESET");
}

// ================ DYNAMIC MODE FUNCTIONS ================
void setDynamicMode(bool enable) {
    dynamicMode = enable;
    Serial.printf("Dynamic mode %s\n", enable ? "ENABLED" : "DISABLED");
    if (dynamicMode) {
        dynamicHeader = buildDynamicCSVHeader();   
        if (loggingActive && sdReady && sessionState == SESSION_STATE_ACTIVE) {
            rotateFile(ROTATE_REASON_USER_COMMAND);
        }
    }
}

void updateDynamicHeader() {
    if (dynamicMode) {
        dynamicHeader = buildDynamicCSVHeader();   
    }
}

String getDynamicHeader() {
    return dynamicHeader;
}

// ================ LOG DYNAMIC DATA TO SD (includes I2C values) ================
void logDynamicDataToSD() {
    if (!sdReady || !dynamicMode) return;
    if (sessionState != SESSION_STATE_ACTIVE) return;
    
    unsigned long now = millis();
    unsigned long elapsed_ms = now - startTime;
    
    char rowBuffer[4096];  
    String ts = getFormattedTime();
    int pos = snprintf(rowBuffer, sizeof(rowBuffer), "%s", ts.c_str());
    
    // ------------------ DBC SIGNALS ------------------
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        for (const auto& pair : activeSignals) {
            for (const auto& sig : pair.second) {
                auto it = lastDynamicValues.find(sig.name);
                if (it != lastDynamicValues.end()) {
                    pos += snprintf(rowBuffer + pos, sizeof(rowBuffer) - pos, 
                                   ",%.3f", it->second);
                } else {
                    if (pos < sizeof(rowBuffer) - 2) rowBuffer[pos++] = ',';
                }
            }
        }
        xSemaphoreGive(dataMutex);
    }
    
    // ------------------ HARDCODED THERMOCOUPLES (SHORT NAMES) ------------------
    String tcNamesShort[] = {
        "TC1", "TC2", "TC3", "TC4", "TC5", "TC6",
        "TC7", "TC8", "TC9", "TC10", "TC11", "TC12"
    };
    
    for (int i = 0; i < 12; i++) {
        auto it = i2cValues.find(tcNamesShort[i]);
        if (it != i2cValues.end()) {
            pos += snprintf(rowBuffer + pos, sizeof(rowBuffer) - pos, ",%.3f", it->second);
        } else {
            if (pos < sizeof(rowBuffer) - 2) rowBuffer[pos++] = ',';
        }
    }
    
    // ------------------ I2C SIGNALS FROM CONFIG (SKIP MCP9600) ------------------
    for (const auto& dev : i2cConfig.devices) {
        if (dev.type == "MCP9600") continue;  
        for (const auto& sig : dev.signals) {
            if (!sig.enabled) continue;
            auto it = i2cValues.find(sig.name);
            if (it != i2cValues.end()) {
                
                if (sig.isMapped) {
                    uint16_t raw = (uint16_t)it->second;
                    auto mapIt = sig.valueMapping.find(raw);
                    if (mapIt != sig.valueMapping.end()) {
                        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%s", mapIt->second.c_str());
                    } else {
                        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.3f", it->second);
                    }
                } else {
                    pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.3f", it->second);
                }
            } else {
                if (pos < sizeof(rowBuffer)-2) rowBuffer[pos++] = ',';
            }
        }
    }

    pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.0f", speedData.rpm);

    // ------------------ GPS DATA ------------------

    if (gpsData.location_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.6f", gpsData.latitude);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }

    if (gpsData.location_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.6f", gpsData.longitude);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }

    if (gpsData.altitude_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.1f", gpsData.altitude);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }
  
    if (gpsData.speed_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.1f", gpsData.speed_kmh);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }

    if (pos < sizeof(rowBuffer) - 1) {
        rowBuffer[pos++] = '\n';
        rowBuffer[pos] = '\0';
    }
    
    addToBuffer(rowBuffer, pos);
      
    loggedCount++;
    incrementSessionRecords();
    incrementFileRecords();
}