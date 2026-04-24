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
#include "i2c_config.h"          // for dynamic I2C config

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
extern std::map<String, double> i2cValues;   // from globals.cpp

// ================ DEBUGGING GLOBALS ================
static unsigned long logCallCount = 0;
static unsigned long lastLogCallPrint = 0;

// ================ FLUSH TASK SYNC ================
extern SemaphoreHandle_t flushSemaphore;   // may still be used elsewhere

// External references (defined in main .ino file)
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

// Forward declarations of functions used
extern bool needsFileRotation();
extern void rotateFile(RotateReason_t reason);
extern void createNewLogFile();
extern void incrementSessionRecords();
extern void incrementFileRecords();

// External CAN data references
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
    // Allocate double buffers
    if (buffers[0] == nullptr) {
        buffers[0] = (char*) malloc(bufferSize);
        buffers[1] = (char*) malloc(bufferSize);

        // Create synchronization primitives
        bufferMutex = xSemaphoreCreateMutex();
        flushQueue = xQueueCreate(2, sizeof(FlushItem_t));   // queue up to 2 buffer items
        bufferFreeSem[0] = xSemaphoreCreateBinary();
        bufferFreeSem[1] = xSemaphoreCreateBinary();
        xSemaphoreGive(bufferFreeSem[0]);   // both buffers initially free
        xSemaphoreGive(bufferFreeSem[1]);
    }

    // Reset state
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
    if (!buffers[0] || !buffers[1]) return;   // not initialized
    if (len <= 0) return;

    xSemaphoreTake(bufferMutex, portMAX_DELAY);

    // Check if current buffer has enough space
    if (bufferIndex + len >= bufferSize - 1) {
        // Current buffer is full – need to swap
        int oldBuffer = activeBufferIndex;
        int newBuffer = (activeBufferIndex + 1) % 2;

        // Wait for the new buffer to be free (not being flushed)
        if (xSemaphoreTake(bufferFreeSem[newBuffer], pdMS_TO_TICKS(1000)) != pdTRUE) {
            // Timeout – both buffers busy? This should be rare. Fallback: block until free.
            Serial.println("⚠️ addToBuffer: waiting for free buffer...");
            xSemaphoreTake(bufferFreeSem[newBuffer], portMAX_DELAY);
        }

        // Queue the old buffer for flushing (store its index and current size)
        FlushItem_t item = {oldBuffer, bufferIndex};
        xQueueSend(flushQueue, &item, 0);

        // Switch to new buffer
        activeBufferIndex = newBuffer;
        bufferIndex = 0;

        // The old buffer's semaphore is still held by the flush task; it will be given back after write.
    }

    // Copy data into current buffer
    memcpy(buffers[activeBufferIndex] + bufferIndex, data, len);
    bufferIndex += len;

    xSemaphoreGive(bufferMutex);
}

// ================ FLUSH TASK (to be called from FreeRTOS task) ================
void flushBufferTask(void *pvParameters) {
    FlushItem_t item;
    const TickType_t maxWait = pdMS_TO_TICKS(2000);   // wait up to 2 seconds for a buffer

    while (1) {
        // Wait for a buffer to be ready in the queue
        if (xQueueReceive(flushQueue, &item, maxWait) == pdTRUE) {
            // We have a full buffer to write
            if (item.bufferIdx < 0 || item.bufferIdx > 1) continue;   // sanity check
            if (item.dataSize <= 0) {
                // Nothing to write – just free the buffer
                xSemaphoreGive(bufferFreeSem[item.bufferIdx]);
                continue;
            }

            // Write the buffer to SD
            if (sdReady && buffers[item.bufferIdx]) {
                // Check if file rotation is needed
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

            // Mark buffer as free for reuse
            xSemaphoreGive(bufferFreeSem[item.bufferIdx]);
        } else {
            // Timeout – no buffer was queued. Optionally flush the current partial buffer? 
            // For simplicity, we do nothing. The main loop will eventually fill the buffer.
        }
    }
}

// ================ LEGACY FUNCTIONS (modified to use new system) ================
void flushBuffer() {
    // Legacy function – now does nothing. Could be used to force a flush of the current
    // active buffer if needed (e.g., before shutdown), but we keep it empty for compatibility.
    // If you need a forced flush, implement a separate function that queues the current buffer.
}

// ================ FORMAT A COMPLETE CSV LINE ================
void formatCompleteCSV(char* buffer, size_t bufferSize, 
                       unsigned long timestampMs, unsigned long currentTime) {
    int pos = 0;
    
    // Timestamp and counters
    String ts = getFormattedTime();
    pos += snprintf(buffer + pos, bufferSize - pos, "%s,%u,%u",
                    ts.c_str(), sessionRecordCounter, fileRecordCounter);
    
    // BATT_ST1 (0x2F4) - Battery pack data
    if (isValid(battSt1.lastUpdate, battSt1.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%.3f,%.3f", 
                        battSt1.soc, battSt1.voltage, battSt1.current);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    // CELL_VOLT (0x4F4) - Cell voltage data
    if (isValid(cellVolt.lastUpdate, cellVolt.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%u,%d,%u",
                        cellVolt.maxCellNo, cellVolt.maxCellVolt,
                        cellVolt.minCellNo, cellVolt.minCellVolt);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // CELL_TEMP (0x5F4) - Cell temperature data
    if (isValid(cellTemp.lastUpdate, cellTemp.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%d,%d,%d,%d",
                        cellTemp.maxCellTemp, cellTemp.maxCtNO,
                        cellTemp.minCellTemp, cellTemp.minCtNO,
                        cellTemp.avgCellTemp);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,");
    }
    
    // BATT_ST2 (0x18F128F4) - Battery stats
    if (isValid(battSt2.lastUpdate, battSt2.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%u",
                        battSt2.capRemain, battSt2.fulChargeCap,
                        battSt2.cycleCap, battSt2.cycleCount);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // BMS_INFO (0x18F428F4) - BMS info
    if (isValid(bmsInfo.lastUpdate, bmsInfo.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%lu,%u,%d",
                        bmsInfo.bmsRunTime, bmsInfo.heatCur, bmsInfo.soh);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    // BMS_6 (0x08F4) - Current limits
    if (isValid(bms6.lastUpdate, bms6.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%.1f",
                        bms6.cdcl, bms6.cccl, bms6.pdcl, bms6.pccl);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // BMS_SW_STA (0x18F528F4) - Switch states
    if (isValid(bmsSwSta.lastUpdate, bmsSwSta.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%d,%d,%d,%d,%d",
                        bmsSwSta.chgMosSta, bmsSwSta.dchgMosSta,
                        bmsSwSta.balanSta, bmsSwSta.heatSta,
                        bmsSwSta.chgDevPlugSta, bmsSwSta.accSta);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,");
    }
    
    // CELL_VOLTAGES (0x18Ex28F4) - Individual cell voltages (first 16 cells)
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
    
    // MCU_MSG_1 (0x102200A0) - Motor controller basic data
    if (isValid(mcuMsg1.lastUpdate, mcuMsg1.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d,%d,%d,%d",
                        mcuMsg1.dcVolt, mcuMsg1.motorTemp,
                        mcuMsg1.cntrlTemp, mcuMsg1.throttlePercent);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // MCU_MSG_2 (0x102200A1) - Motor controller speed data
    if (isValid(mcuMsg2.lastUpdate, mcuMsg2.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%u,%u,%u",
                        mcuMsg2.motorSpeed, mcuMsg2.motorSpdLim, mcuMsg2.speedMode);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    // AUX_MOTOR_1 (0x19FF50F0) - Aux motor estimated data
    if (isValid(auxMotor1.lastUpdate, auxMotor1.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%u,%d,%d",
                        auxMotor1.torqueEst, auxMotor1.speedEst,
                        auxMotor1.controllerTemp, auxMotor1.motorTemp);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // AUX_MOTOR_2 (0x19FF50F1) - Aux motor electrical data
    if (isValid(auxMotor2.lastUpdate, auxMotor2.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%u",
                        auxMotor2.motorCurrent, auxMotor2.motorVoltage,
                        auxMotor2.mcuVoltage, auxMotor2.canLife);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // CHRG_OUT (0x18FF50E5) - Charger output data
    if (isValid(chrgOut.lastUpdate, chrgOut.timeoutMs, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f,%d",
                        chrgOut.chargerVoltageOut, chrgOut.chargerCurrentOut,
                        chrgOut.chargerInputAcVolt, chrgOut.chargerInternalTemp);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
    }
    
    // ================ GPS DATA SECTION ================
    // Add debug to see if GPS data is valid (prints every 10 seconds)
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
    
    // Latitude, Longitude
    if (gpsData.location_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.6f,%.6f", 
                        gpsData.latitude, gpsData.longitude);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,");
    }
    
    // Altitude
    if (gpsData.altitude_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f", gpsData.altitude);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Speed (km/h, m/s, knots)
    if (gpsData.speed_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f", 
                        gpsData.speed_kmh, gpsData.speed_mps, gpsData.speed_knots);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    // Course and cardinal direction
    if (gpsData.course_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%s", 
                        gpsData.course_deg, gpsData.cardinal_direction.c_str());
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,");
    }
    
    // Time (UTC)
    if (gpsData.time_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%02d:%02d:%02d",
                        gpsData.hour_utc, gpsData.minute_utc, gpsData.second_utc);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Date
    if (gpsData.date_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%04d-%02d-%02d",
                        gpsData.year, gpsData.month, gpsData.day);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Time (IST - Indian Standard Time)
    if (gpsData.time_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%02d:%02d:%02d",
                        gpsData.hour_ist, gpsData.minute_ist, gpsData.second_utc);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Satellites and HDOP
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
    
    // Compass data
    if (compassData.valid && isValid(compassData.lastUpdate, 5000, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%s,%.1f,%.1f,%.1f",
                        compassData.heading_deg, compassData.cardinal_direction.c_str(),
                        compassData.raw_x, compassData.raw_y, compassData.raw_z);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,");
    }
    
    // GPS Statistics
    unsigned long uptime_seconds = (currentTime - gpsStats.uptime_ms) / 1000;
    pos += snprintf(buffer + pos, bufferSize - pos, ",%lu,%.1f,%.3f",
                    uptime_seconds, gpsStats.maxSpeed_kmh, gpsStats.totalDistance_km);


// ================ SPEED SENSOR SECTION ================
  // Speed Sensor Data (Hall effect)
  if (speedData.valid && isValid(speedData.lastUpdate, speedData.timeoutMs, currentTime)) {
      pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.0f,%.1f,%.1f",
                      speedData.frequency_hz, speedData.rpm, 
                      speedData.speed_kmh, speedData.speed_mps);
  } else {
      pos += snprintf(buffer + pos, bufferSize - pos, ",,,,");
  }
      
// ================ I2C SENSORS DATA SECTION (static mode) ================
    formatI2CData(buffer + pos, bufferSize - pos, currentTime);
    
    // End of line
    snprintf(buffer + pos, bufferSize - pos, "\n");
}

// ================ LOG DATA TO SD (called every 100ms) ================
void logDataToSD() {
    logCallCount++;   // DEBUG: count each call

    if (!sdReady) return;
    if (sessionState != SESSION_STATE_ACTIVE) return;

    // ----- ensure a log file exists -----
    if (strlen(currentFilePath) == 0) {   // no file open yet
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

    // DEBUG: print call frequency every 5 seconds
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
        dynamicHeader = buildDynamicCSVHeader();   // from dynamic_decoder.h
        if (loggingActive && sdReady && sessionState == SESSION_STATE_ACTIVE) {
            rotateFile(ROTATE_REASON_USER_COMMAND);
        }
    }
}

void updateDynamicHeader() {
    if (dynamicMode) {
        dynamicHeader = buildDynamicCSVHeader();   // from dynamic_decoder.h
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
    
    char rowBuffer[4096];  // Increased buffer size for all data
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
        if (dev.type == "MCP9600") continue;  // already handled by hardcoded section
        for (const auto& sig : dev.signals) {
            if (!sig.enabled) continue;
            auto it = i2cValues.find(sig.name);
            if (it != i2cValues.end()) {
                // If the signal has a value mapping, output the label instead of the number
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

        // Speed Sensor
    if (speedData.valid && isValid(speedData.lastUpdate, speedData.timeoutMs, now)) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.0f", speedData.rpm);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }

    // ------------------ GPS DATA ------------------
    // Latitude
    if (gpsData.location_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.6f", gpsData.latitude);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }
    // Longitude
    if (gpsData.location_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.6f", gpsData.longitude);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }
    // Altitude
    if (gpsData.altitude_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.1f", gpsData.altitude);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }
    // Speed (km/h)
    if (gpsData.speed_valid) {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",%.1f", gpsData.speed_kmh);
    } else {
        pos += snprintf(rowBuffer + pos, sizeof(rowBuffer)-pos, ",");
    }

    // End the line
    if (pos < sizeof(rowBuffer) - 1) {
        rowBuffer[pos++] = '\n';
        rowBuffer[pos] = '\0';
    }
    
    addToBuffer(rowBuffer, pos);
      
    loggedCount++;
    incrementSessionRecords();
    incrementFileRecords();
}