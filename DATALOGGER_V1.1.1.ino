/*
 * COMPLETE BMS DATA LOGGER WITH USB TTL CONTROL v3.0
 * Modular Architecture – with GPS and I2C Sensor integration
 */

#include "config.h"
#include "pins.h"
#include "types.h"
#include "globals.h"
#include "utils.h"
#include "can_decoder.h"
#include "ecu_state.h"
#include "session_manager.h"
#include "file_manager.h"
#include "data_logger.h"
#include "sd_card.h"
#include "ui_handler.h"
#include "wifi_manager.h"
#include "dynamic_decoder.h"
#include <SPIFFS.h>
#include <set>
#include <map>
#include <ArduinoJson.h>
#include "gps_manager.h"
#include "gps_globals.h"
#include "dbc_parser.h"
#include "signal_selector.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "i2c_sensors.h"
#include "i2c_task.h"
#include "speed_sensor.h"

SemaphoreHandle_t dataMutex = NULL;
extern std::map<uint32_t, std::vector<DBCSignal>> activeSignals;
extern std::map<String, double> lastDynamicValues;
extern bool dynamicMode;
extern HardwareSerial gpsSerial;

// ================ InfluxDB timing globals ================
SemaphoreHandle_t influxStatsMutex = NULL;
unsigned long influxTotalTime = 0;
unsigned long influxCount = 0;

// ================ Flush task globals ================
SemaphoreHandle_t flushSemaphore = NULL;
TaskHandle_t flushTaskHandle = NULL;

// ================ FORWARD DECLARATIONS ================
void flushTask(void *pvParameters);
void statsTask(void *pvParameters);
void gpsTask(void *pvParameters);

// ================ DEBUGGING GLOBALS ================
static unsigned long lastDebugPrint = 0;
static unsigned long loopCounter = 0;
static unsigned long maxLoopTime = 0;
static unsigned long totalLoopTime = 0;

static unsigned long totalCAN = 0, totalWeb = 0, totalECU = 0, totalLog = 0, totalFlush = 0, totalStats = 0;
static unsigned long countCAN = 0, countWeb = 0, countECU = 0, countLog = 0, countFlush = 0, countStats = 0;

static ECUState_t oldEcuState = ECU_STATE_UNKNOWN;
static SessionState_t oldSessionState = SESSION_STATE_BOOT;

// ================ RTOS TASK HANDLE ================
TaskHandle_t influxTaskHandle = NULL;

// ================ FUNCTION PROTOTYPES ================
void processCANMessages();
void handleEmergencyShutdown();
void influxTask(void *pvParameters);   // FreeRTOS task for InfluxDB

// ================ LOAD DBC CONFIGURATION ================
void loadDynamicConfig() {
    if (!SPIFFS.exists(SIGNAL_CONFIG_PATH)) {
        Serial.println("No signal selection found, staying in static mode");
        return;
    }

    std::vector<DBCMessage> messages;
    if (!loadDBCMessagesFromJson(messages, "/dbc/messages.json")) {
        Serial.println("Failed to load DBC messages");
        return;
    }

    File file = SPIFFS.open(SIGNAL_CONFIG_PATH, FILE_READ);
    if (!file) {
        Serial.println("Failed to open signal config");
        return;
    }
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
        Serial.println("Failed to parse signal config");
        return;
    }

    std::map<uint32_t, std::set<String>> selectedMap;
    JsonArray arr = doc.as<JsonArray>();
    for (JsonObject obj : arr) {
        uint32_t id = obj["id"];
        String sigName = obj["signal"].as<String>();
        selectedMap[id].insert(sigName);
    }

    std::map<uint32_t, std::vector<DBCSignal>> activeMap;
    for (auto& msg : messages) {
        std::vector<DBCSignal> selectedSignals;
        for (auto& sig : msg.signals) {
            auto it = selectedMap.find(msg.id);
            if (it != selectedMap.end() && it->second.count(sig.name)) {
                sig.isSelected = true;
                selectedSignals.push_back(sig);
            }
        }
        if (!selectedSignals.empty()) {
            activeMap[msg.id] = selectedSignals;
        }
    }

    if (activeMap.empty()) {
        Serial.println("No signals selected");
        return;
    }

    setActiveSignals(activeMap);
    initDynamicValues(activeMap);
    setDynamicMode(true);
    Serial.println("Dynamic mode restored from saved config");
}

// ================ SETUP ================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial2.begin(921600, SERIAL_8N1, UI_RXD2, UI_TXD2);
    while (Serial2.available()) Serial2.read();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    initializeTimeouts();

    initUI();
    initSD();

    // Mount SPIFFS first
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ SPIFFS Mount Failed!");
    } else {
        if (SPIFFS.exists("/dbc.html")) {
            SPIFFS.remove("/dbc.html");
        }
        
        // Create config directory if it doesn't exist
        if (!SPIFFS.exists("/config")) {
            SPIFFS.mkdir("/config");
        }
        
        // Load configuration from SPIFFS
        loadConfigFromSPIFFS();
    }
    
    createDBCFileIfNeeded();
    SPIFFS.mkdir("/config");

    dataMutex = xSemaphoreCreateMutex();

    // Load DBC configuration only
    loadDynamicConfig();

    influxStatsMutex = xSemaphoreCreateMutex();
    flushSemaphore = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(i2cTask, "I2C Task", 4096, NULL, 1, NULL, 0);

    if (sdReady) {
        initFileManager();
        initSessionManager();
    }

    initWiFi();
    startWebServer();

    // Initialize data logger
    initDataLogger();

    // Create the InfluxDB task on Core 0
    xTaskCreatePinnedToCore(
        influxTask,
        "InfluxTask",
        8192,
        NULL,
        1,
        &influxTaskHandle,
        0
    );

    // Create the flush task on Core 0
    xTaskCreatePinnedToCore(
        flushTask,
        "FlushTask",
        4096,
        NULL,
        1,
        &flushTaskHandle,
        0
    );
    
    delay(10);
    Serial2.println(sdReady ? "SD_OK" : "SD_ERROR");
    delay(10);
    Serial2.println("WIFI_" + wifiStatus);
    if (wifiIP.length() > 0) {
        Serial2.println("IP_" + wifiIP);
    }
    delay(10);

    initCAN();
    initECUState();
    initSpeedSensor();

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    // Set timezone to IST (UTC+5:30)
    setenv("TZ", "IST-5:30", 1);
    tzset();
}

// ================ MAIN LOOP ================
void loop() {
    unsigned long loopStart = micros();

    // --- Web server handling ---
    unsigned long t1 = micros();
    handleWebServer();
    unsigned long t2 = micros();
    totalWeb += (t2 - t1);
    countWeb++;

    unsigned long now = millis();

    // Process UI commands
    processUICommands();

    // --- CAN message processing ---
    t1 = micros();
    processCANMessages();
    t2 = micros();
    totalCAN += (t2 - t1);
    countCAN++;
    
    // ===== 30‑second inactivity timeout =====
    if (sessionState == SESSION_STATE_ACTIVE) {
        if (lastFilteredTime != 0 && (millis() - lastFilteredTime) > 30000) {
            Serial.println("No filtered data for 30s, closing file...");
            rotateFile(ROTATE_REASON_ECU_DISCONNECT);
            sessionState = SESSION_STATE_WAITING;
        }
    }
    // ========================================

    // --- ECU state check ---
    t1 = micros();
    checkECUState();
    t2 = micros();
    totalECU += (t2 - t1);
    countECU++;
    updateSpeed();

    // Log data if active
    if (loggingActive && sdReady && now - lastLogTime >= logIntervalMs) {
        t1 = micros();
        lastLogTime = now;
        logDataToSD();
        t2 = micros();
        totalLog += (t2 - t1);
        countLog++;
    }

    // Manual commands from Serial Monitor
    if (Serial.available()) {
        String usbCmd = Serial.readStringUntil('\n');
        usbCmd.trim();
        if (usbCmd.length() > 0) {
            Serial.print("\n📤 [Manual] Sending to UI: ");
            Serial.println(usbCmd);
            Serial2.println(usbCmd);
        }
    }

    // Emergency check
    // In loop() - change MAX_FILE_SIZE to maxFileSizeMB
   if (sdReady && (ESP.getFreeHeap() < 10000 || currentFileSize > (uint64_t)maxFileSizeMB * 1024 * 1024 * 1.2)) {
        handleEmergencyShutdown();
    }

    // ECU and session state change tracking
    if (ecuState != oldEcuState) {
        Serial.printf("ECU state changed: %d -> %d\n", oldEcuState, ecuState);
        oldEcuState = ecuState;
    }
    if (sessionState != oldSessionState) {
        Serial.printf("Session state changed: %d -> %d\n", oldSessionState, sessionState);
        oldSessionState = sessionState;
    }

    // --- End of loop timing ---
    unsigned long loopEnd = micros();
    unsigned long loopDuration = loopEnd - loopStart;

    loopCounter++;
    totalLoopTime += loopDuration;
    if (loopDuration > maxLoopTime) maxLoopTime = loopDuration;

    // Print debug summary every 5 seconds
    if (millis() - lastDebugPrint > 5000) {
        Serial.printf("\n--- DEBUG SUMMARY (5s) ---\n");
        Serial.printf("Loop: avg=%lu µs, max=%lu µs, count=%lu\n",
                      totalLoopTime / loopCounter, maxLoopTime, loopCounter);
        if (countCAN) Serial.printf("  CAN: avg=%lu µs\n", totalCAN / countCAN);
        if (countWeb) Serial.printf("  Web: avg=%lu µs\n", totalWeb / countWeb);
        if (countECU) Serial.printf("  ECU: avg=%lu µs\n", totalECU / countECU);
        if (countLog) Serial.printf("  Log: avg=%lu µs\n", totalLog / countLog);
        if (countFlush) Serial.printf("  Flush: avg=%lu µs\n", totalFlush / countFlush);
        if (countStats) Serial.printf("  Stats: avg=%lu µs\n", totalStats / countStats);

        if (influxCount > 0) {
            unsigned long avgInflux = 0;
            if (xSemaphoreTake(influxStatsMutex, portMAX_DELAY) == pdTRUE) {
                avgInflux = influxTotalTime / influxCount;
                influxTotalTime = 0;
                influxCount = 0;
                xSemaphoreGive(influxStatsMutex);
            }
            Serial.printf("  Influx: avg=%lu µs\n", avgInflux);
        }

        Serial.printf("----------------------------\n");

        lastDebugPrint = millis();
        loopCounter = 0;
        totalLoopTime = 0;
        maxLoopTime = 0;
        totalCAN = totalWeb = totalECU = totalLog = totalFlush = totalStats = 0;
        countCAN = countWeb = countECU = countLog = countFlush = countStats = 0;
    }

    delay(1);
}

void processCANMessages() {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
        messageCount++;
        lastCANActivity = millis();

        if (dynamicMode) {
            auto it = activeSignals.find(message.identifier);
            if (it != activeSignals.end()) {
                decodeDynamic(message, lastDynamicValues);
            }
            digitalWrite(LED_PIN, HIGH);
            delayMicroseconds(20);
            digitalWrite(LED_PIN, LOW);
        } else {
            if (acceptCANId(message.identifier, message.extd)) {
                acceptedCount++;

                if (!message.extd) {
                    switch (message.identifier) {
                        case 0x2F4: decodeBattSt1(message); break;
                        case 0x4F4: decodeCellVolt(message); break;
                        case 0x5F4: decodeCellTemp(message); break;
                        case 0x7F4: decodeAlmInfo(message); break;
                        default: break;
                    }
                } else {
                    switch (message.identifier) {
                        case 0x08F4: decodeBms6(message); break;
                        case 0x18F128F4: decodeBattSt2(message); break;
                        case 0x18F228F4: decodeAllTemp(message); break;
                        case 0x18F328F4: decodeBmsErrInfo(message); break;
                        case 0x18F428F4: decodeBmsInfo(message); break;
                        case 0x18F528F4: decodeBmsSwSta(message); break;
                        case 0x18E028F4: decodeCellVoltage(message, 1); break;
                        case 0x18E128F4: decodeCellVoltage(message, 5); break;
                        case 0x18E228F4: decodeCellVoltage(message, 9); break;
                        case 0x18E328F4: decodeCellVoltage(message, 13); break;
                        case 0x18E428F4: decodeCellVoltage(message, 17); break;
                        case 0x18E528F4: decodeCellVoltage(message, 21); break;
                        case 0x18E628F4: decodeCellVoltage(message, 25); break;
                        case 0x1806E5F4: decodeBmsChgInfo(message); break;
                        case 0x18F0F428: decodeCtrlInfo(message); break;
                        case 0x102200A0: decodeMcuMsg1(message); break;
                        case 0x102200A1: decodeMcuMsg2(message); break;
                        case 0x102200A2: decodeMcuMsg3(message); break;
                        case 0x102200A3: decodeMcuMsg4(message); break;
                        case 0x102200A4: decodeMcuMsg5(message); break;
                        case 0x102200A5: decodeMcuMsg6(message); break;
                        case 0x19FF50F0: decodeAuxMotor1(message); break;
                        case 0x19FF50F1: decodeAuxMotor2(message); break;
                        case 0x19FF50F2: decodeAuxMotor3(message); break;
                        case 0x18FF50E5: decodeChrgOut(message); break;
                        default: break;
                    }
                }

                digitalWrite(LED_PIN, HIGH);
                delayMicroseconds(20);
                digitalWrite(LED_PIN, LOW);

            } else {
                filteredOutCount++;
            }
        }
    }
}

// ================ EMERGENCY HANDLING ================
void handleEmergencyShutdown() {
    Serial.println("EMERGENCY SHUTDOWN!");

    flushBuffer();
    closeCurrentFile(ROTATE_REASON_SYSTEM);

    char emergencyPath[64];
    snprintf(emergencyPath, sizeof(emergencyPath), "/temp/emergency_%lu.csv", millis());

    File emergencyFile = SD.open(emergencyPath, FILE_WRITE);
    if (emergencyFile) {
        emergencyFile.println("EMERGENCY SHUTDOWN DATA");
        emergencyFile.printf("Last session: %d\n", currentSession.sessionId);
        emergencyFile.printf("Last file: %d\n", currentSession.fileSequence);
        emergencyFile.printf("Last session record: %d\n", sessionRecordCounter);
        emergencyFile.printf("Last file record: %d\n", fileRecordCounter);
        emergencyFile.printf("ECU State: %d\n", ecuState);
        emergencyFile.close();
    }
}

// ================ FREERTOS TASK FOR INFLUXDB ================
void influxTask(void *pvParameters) {
    const TickType_t interval = pdMS_TO_TICKS(100);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        sendInfluxDBData();
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

// ================ FLUSH TASK ================
void flushTask(void *pvParameters) {
    flushBufferTask(pvParameters);
}

// ================ GPS TASK ================
void gpsTask(void *pvParameters) {
    // Initialize GPS and compass
    initGPS();
    initCompass();
    
    const TickType_t interval = pdMS_TO_TICKS(100);
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    bool timeSetFromGPS = false;
    unsigned long lastTimeSyncAttempt = 0;
    const unsigned long TIME_SYNC_RETRY_INTERVAL = 30000; // Try every 30 seconds
    unsigned long lastSerialPrint = 0;   // For periodic Serial output
    unsigned long lastRawPrint = 0;      // For raw NMEA output
    bool printRawNMEA = false;           // Toggle to enable raw NMEA
    
    while (1) {
        updateGPS();
        updateCompass();
        
        unsigned long now = millis();
        
        // Try to set time from GPS if not already set
        if (!timeSetFromGPS && gpsData.time_valid && gpsData.date_valid) {
            // Validate year (must be reasonable)
            if (gpsData.year >= 2020 && gpsData.year <= 2030) {
                struct tm tm;
                tm.tm_year = gpsData.year - 1900;
                tm.tm_mon = gpsData.month - 1;
                tm.tm_mday = gpsData.day;
                tm.tm_hour = gpsData.hour_utc;
                tm.tm_min = gpsData.minute_utc;
                tm.tm_sec = gpsData.second_utc;
                tm.tm_isdst = -1;
                
                time_t t = mktime(&tm);
                if (t > 1609459200) { // > 2021-01-01
                    struct timeval tv = { t, 0 };
                    settimeofday(&tv, NULL);
                    
                    Serial.printf("✅ System time set from GPS: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                                 gpsData.year, gpsData.month, gpsData.day,
                                 gpsData.hour_utc, gpsData.minute_utc, gpsData.second_utc);
                    
                    // Set timezone to IST
                    setenv("TZ", "IST-5:30", 1);
                    tzset();
                    
                    timeSetFromGPS = true;
                    Serial2.println("TIME_SYNC_OK");
                    
                    // Log local time for verification
                    time_t now_local = time(nullptr);
                    struct tm *local = localtime(&now_local);
                    Serial.printf("Local time (IST): %02d:%02d:%02d\n",
                                 local->tm_hour, local->tm_min, local->tm_sec);
                }
            }
            lastTimeSyncAttempt = now;
        }
        
        // Periodically remind that we're waiting for GPS time
        if (!timeSetFromGPS && now - lastTimeSyncAttempt > TIME_SYNC_RETRY_INTERVAL) {
            if (gpsData.time_valid) {
                Serial.println("⏳ GPS time available but date invalid?");
            } else {
                Serial.println("⏳ Still waiting for GPS time...");
            }
            lastTimeSyncAttempt = now;
        }

        // ========== Print raw NMEA sentences every 5 seconds (toggle with command) ==========
        if (printRawNMEA && (now - lastRawPrint > 5000)) {
            Serial.println("\n--- Raw NMEA Data (last 5 seconds) ---");
            // Dump the GPS serial buffer (non-destructive)
            while (gpsSerial.available()) {
                char c = gpsSerial.read();
                Serial.print(c);
            }
            Serial.println("\n--- End of NMEA ---\n");
            lastRawPrint = now;
        }
        
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

// ================ I2C TASK ================
void i2cTask(void *pvParameters) {
    // Initialize I2C sensors
    initI2CSensors();
    
    const TickType_t interval = pdMS_TO_TICKS(I2C_UPDATE_INTERVAL_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1) {
        updateI2CSensors();
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

void statsTask(void *pvParameters) {
    const TickType_t interval = pdMS_TO_TICKS(5000);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        sendStats();
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}