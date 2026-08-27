/*
 * COMPLETE BMS DATA LOGGER WITH USB TTL CONTROL v1.3.4
 * - UI Mode implementation: pauses logging and MQTT when UI is active
 * - Automatic timeout after 5 seconds of inactivity
 * - Forced UI mode with "ui on" / "ui off" commands
 * - Mutex-protected Serial1 writes
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
#include "speed_sensor.h"
#include "ads1115_scheduler.h"

SemaphoreHandle_t dataMutex = NULL;

// ================ EXTERN DECLARATIONS ================
extern std::map<String, double> lastDynamicValues;
extern std::map<uint32_t, std::vector<DBCSignal>> activeSignals;
extern bool dynamicMode;

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
void webServerTask(void *pvParameters);
void uiTask(void *pvParameters);

// ================ DEBUGGING GLOBALS ================
static unsigned long lastDebugPrint = 0;
static unsigned long loopCounter = 0;
static unsigned long maxLoopTime = 0;
static unsigned long totalLoopTime = 0;

static unsigned long totalCAN = 0, totalWeb = 0, totalECU = 0, totalLog = 0, totalFlush = 0, totalStats = 0;
static unsigned long countCAN = 0, countWeb = 0, countECU = 0, countLog = 0, countFlush = 0, countStats = 0;

static ECUState_t oldEcuState = ECU_STATE_UNKNOWN;
static SessionState_t oldSessionState = SESSION_STATE_BOOT;

// ================ FUNCTION PROTOTYPES ================
void processCANMessages();
void handleEmergencyShutdown();

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

    // ============================================================
    // UI on Serial1 (GPIO14, GPIO15) - Hardware UART1
    // ============================================================
    Serial1.begin(921600, SERIAL_8N1, UI_RX_PIN, UI_TX_PIN);
    Serial1.setTimeout(10);
    while (Serial1.available()) Serial1.read();

    // ============================================================
    // Serial2 remains for torque sensor on GPIO25, GPIO26
    // ============================================================
    // Do NOT call Serial2.begin() here – torque_begin() will handle it.

    // Create the I2C mutex early
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("❌ Failed to create I2C mutex!");
    }

    // Create UI mode mutex
    uiModeMutex = xSemaphoreCreateMutex();
    if (uiModeMutex == NULL) {
        Serial.println("❌ Failed to create UI mode mutex!");
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    initializeTimeouts();

    initUI();  // Now uses Serial1 on GPIO14/15
    initSD();

    if (!SPIFFS.begin(true)) {
        Serial.println("❌ SPIFFS Mount Failed!");
    } else {
        if (SPIFFS.exists("/dbc.html")) {
            SPIFFS.remove("/dbc.html");
        }
        if (!SPIFFS.exists("/config")) {
            SPIFFS.mkdir("/config");
        }
        loadConfigFromSPIFFS();
    }

    createDBCFileIfNeeded();
    SPIFFS.mkdir("/config");

    dataMutex = xSemaphoreCreateMutex();
    loadDynamicConfig();

    // ================ CREATE NEW MUTEXES FOR CROSS-CORE DATA PROTECTION ================
    i2cValuesMutex = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();

    influxStatsMutex = xSemaphoreCreateMutex();
    flushSemaphore = xSemaphoreCreateBinary();

    // --- Initialise I2C sensors and ADC scheduler ---
    initI2CSensors();
    initADS1115Scheduler();

    // --- TORQUE SENSOR (Serial2, 115200, read-only, no DE/RE) ---
    torqueMutex = xSemaphoreCreateMutex();
    if (torque_begin() != ESP_OK) {
        Serial.println("❌ Torque sensor init failed!");
    } else {
        Serial.println("✅ Torque sensor ready on Serial2 (pins 25,26)");
    }

    // --- Initialise GPS and compass ---
    initGPS();  // Now uses SoftwareSerial on GPIO16, GPIO17
    initCompass();

    // --- File and session management ---
    if (sdReady) {
        initFileManager();
        initSessionManager();
    }

    initWiFi();

    // MQTT task
    xTaskCreatePinnedToCore(
        mqttTask,
        "MQTTTask",
        8192,
        NULL,
        1,
        NULL,
        1
    );
    startWebServer();

    initDataLogger();

    // --- Web server task on Core 1 ---
    xTaskCreatePinnedToCore(
        webServerTask,
        "WebServer",
        8192,
        NULL,
        1,
        NULL,
        1
    );

    // --- UI command task on Core 1 (uses Serial1 now) ---
    xTaskCreatePinnedToCore(
        uiTask,
        "UITask",
        16384,
        NULL,
        1,
        NULL,
        1
    );

    // --- Flush task (SD write) on Core 1 ---
    xTaskCreatePinnedToCore(
        flushTask,
        "FlushTask",
        4096,
        NULL,
        1,
        &flushTaskHandle,
        1
    );

    delay(10);

    initCAN();
    initCTSensors();
    initECUState();
    initSpeedSensor();
    initAuxSpeedSensor();

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "IST-5:30", 1);
    tzset();
}

// ================ MAIN LOOP – Now only real-time tasks ================
void loop() {
    unsigned long loopStart = micros();

    unsigned long now = millis();

    // ---- CAN and ECU processing (Core 0) ----
    unsigned long t1 = micros();
    processCANMessages();
    unsigned long t2 = micros();
    totalCAN += (t2 - t1);
    countCAN++;

    if (sessionState == SESSION_STATE_ACTIVE) {
        if (lastFilteredTime != 0 && (millis() - lastFilteredTime) > 30000) {
            Serial.println("No filtered data for 30s, closing file...");
            rotateFile(ROTATE_REASON_ECU_DISCONNECT);
            sessionState = SESSION_STATE_WAITING;
        }
    }

    t1 = micros();
    checkECUState();
    t2 = micros();
    totalECU += (t2 - t1);
    countECU++;
    updateSpeed();
    updateAuxSpeed();

    // Update CT sensors liveness
    updateCTSensors();

    // ---- Data logging (add to buffer) ----
    // Only if UI is NOT active
    if (!isUIActive() && loggingActive && sdReady && now - lastLogTime >= logIntervalMs) {
        t1 = micros();
        lastLogTime = now;
        logDataToSD();
        t2 = micros();
        totalLog += (t2 - t1);
        countLog++;
    }

    // ---- Emergency checks ----
    if (sdReady && (ESP.getFreeHeap() < 10000 || currentFileSize > (uint64_t)maxFileSizeMB * 1024 * 1024 * 1.2)) {
        handleEmergencyShutdown();
    }

    // ---- State change detection ----
    if (ecuState != oldEcuState) {
        Serial.printf("ECU state changed: %d -> %d\n", oldEcuState, ecuState);
        oldEcuState = ecuState;
    }
    if (sessionState != oldSessionState) {
        Serial.printf("Session state changed: %d -> %d\n", oldSessionState, sessionState);
        oldSessionState = sessionState;
    }

    unsigned long loopEnd = micros();
    unsigned long loopDuration = loopEnd - loopStart;

    loopCounter++;
    totalLoopTime += loopDuration;
    if (loopDuration > maxLoopTime) maxLoopTime = loopDuration;

    delay(1);
}

// ================ CAN MESSAGE PROCESSING ================
void processCANMessages() {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
        messageCount++;
        lastCANActivity = millis();

        processCTCANMessage(message);
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

// ================ WEB SERVER TASK (Core 1) ================
void webServerTask(void *pvParameters) {
    while (1) {
        handleWebServer();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================ UI TASK (Core 1) ================
void uiTask(void *pvParameters) {
    while (1) {
        processUICommands();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ================ FLUSH TASK ================
void flushTask(void *pvParameters) {
    flushBufferTask(pvParameters);
}

// ================ STATS TASK (optional) ================
void statsTask(void *pvParameters) {
    const TickType_t interval = pdMS_TO_TICKS(5000);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        sendStats();
        vTaskDelayUntil(&lastWakeTime, interval);
    }
}