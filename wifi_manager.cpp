#include "wifi_manager.h"
#include "sd_card.h"
#include "session_manager.h"
#include "ecu_state.h"
#include "data_logger.h"
#include "can_decoder.h"
#include "file_manager.h"
#include "utils.h"
#include "globals.h"
#include "gps_globals.h"
#include "i2c_sensors.h"
#include "i2c_config.h"            
#include <SPIFFS.h>
#include <map>
#include <vector>
#include <set>
#include <ArduinoJson.h>
#include "dbc_parser.h"
#include "signal_selector.h"
#include "dynamic_decoder.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

WebServer server(80);
String wifiStatus = "Disconnected";
String wifiIP = "";

static File uploadFile;          


// ================ MQTT Client ================
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
bool mqttReady = false;
unsigned long lastMQTTSend = 0;
const unsigned long MQTT_INTERVAL = 1000; 

#define DYNAMIC_SIGNAL_TIMEOUT_MS 2000

// ================ Timing statistics ================
extern SemaphoreHandle_t influxStatsMutex;
extern unsigned long influxTotalTime;
extern unsigned long influxCount;
extern std::map<String, double> i2cValues;

extern String wifiSSID;
extern String wifiPassword;

extern const char* ap_ssid;
extern const char* ap_password;
extern BattSt1_t battSt1;
extern McuMsg1_t mcuMsg1;
extern McuMsg2_t mcuMsg2;
extern CellVolt_t cellVolt;
extern ECUState_t ecuState;
extern uint32_t currentSessionId;
extern uint32_t currentFileSequence;
extern uint32_t sessionRecordCounter;
extern uint32_t fileRecordCounter;
extern bool sdReady;
extern unsigned long messageCount;
extern unsigned long acceptedCount;
extern unsigned long filteredOutCount;
extern unsigned long loggedCount;
extern unsigned long startTime;
extern SessionState_t sessionState;


extern bool dynamicMode;
extern std::map<String, double> lastDynamicValues;
extern SemaphoreHandle_t dataMutex;

extern I2CSensorData sensorData;  

void handleDBCUploadFile();
void handleDBCUpload();
void handleDBCParse();
void handleDBCSave();
void handleDBCStatus();
void handleDBCDelete();

void handleI2CGetConfig();
void handleI2CSaveConfig();
void handleI2CConfigPage();
void createI2CConfigHTML();

void handleI2CUploadFile();
void handleI2CUpload();
void handleI2CDownloadCSV();

// ================ MQTT Connection ================
void connectMQTT() {
    int retries = 0;
    espClient.setInsecure();
    mqttClient.setClient(espClient);

    while (!mqttClient.connected() && retries < 3) {
        Serial.print("Connecting to MQTT broker...");
        if (mqttClient.connect(mqttClientId.c_str(), 
                               mqttUsername.c_str(), 
                               mqttPassword.c_str())) {
            Serial.println("✅ connected");
            mqttReady = true;
            return;
        } else {
            int state = mqttClient.state();
            Serial.printf("❌ failed, rc=%d\n", state);
            retries++;
            if (retries < 3) {
                Serial.println("Retrying in 2s...");
                delay(2000);
            }
        }
    }
    Serial.println("MQTT connection failed after 3 attempts");
    mqttReady = false;
}

// ================ Send data via MQTT ================
void sendMQTTData() {
    unsigned long start = micros();
    unsigned long now = millis();

    if (now - lastMQTTSend < MQTT_INTERVAL) return;

    if (WiFi.status() != WL_CONNECTED) {
        mqttReady = false;
        return;
    }

    if (!mqttReady || !mqttClient.connected()) {
        connectMQTT();
        if (!mqttReady) return;
    }

    mqttClient.loop();

    String payload = "tractor_sensors,vehicle=tractor_01,session=" + String(currentSessionId);
    bool dataAdded = false;

    auto addField = [&](bool valid, double value, const String& fieldName) {
        if (dataAdded) payload += ",";
        else { payload += " "; dataAdded = true; }
        payload += fieldName + "=" + (valid ? String(value, 3) : "-9999");
    };

    if (dynamicMode) {
        std::map<String, double> localValues;
        std::map<String, unsigned long> localTimes;
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            localValues = lastDynamicValues;
            localTimes  = lastDynamicUpdateTime;
            xSemaphoreGive(dataMutex);
        }

        unsigned long now = millis();
        for (const auto& pair : localValues) {
            auto itTime = localTimes.find(pair.first);
            bool fresh = (itTime != localTimes.end() && (now - itTime->second) <= DYNAMIC_SIGNAL_TIMEOUT_MS);
            double val = fresh ? pair.second : 0.0;   
            addField(true, val, pair.first);
        }

        unsigned long i2cAge = now - sensorData.lastScanTime;
        bool i2cFresh = (i2cAge <= DYNAMIC_SIGNAL_TIMEOUT_MS);
        for (const auto& pair : i2cValues) {
            double val = i2cFresh ? pair.second : 0.0;
            addField(true, val, pair.first);
        }

        // ---- GPS data ----
      if (gpsInitialized) {
          if (gpsData.location_valid) {
              addField(true, gpsData.latitude, "G-Lat");
              addField(true, gpsData.longitude, "G-Long");
          }
          if (gpsData.altitude_valid) {
              addField(true, gpsData.altitude, "G-alt");
          }
          if (gpsData.speed_valid) {
              addField(true, gpsData.speed_kmh, "G-spd");
          }
      }
        
        addField(true, speedData.rpm, "speed_rpm");
        
        
    } else {
        addField(isValid(battSt1.lastUpdate, battSt1.timeoutMs, now), battSt1.voltage, "voltage");
        addField(isValid(battSt1.lastUpdate, battSt1.timeoutMs, now), battSt1.current, "current");
        addField(isValid(battSt1.lastUpdate, battSt1.timeoutMs, now), battSt1.soc, "soc");
        addField(isValid(mcuMsg1.lastUpdate, mcuMsg1.timeoutMs, now), mcuMsg1.motorTemp, "motor_temp");
        addField(isValid(mcuMsg1.lastUpdate, mcuMsg1.timeoutMs, now), mcuMsg1.cntrlTemp, "controller_temp");
        addField(isValid(mcuMsg1.lastUpdate, mcuMsg1.timeoutMs, now), mcuMsg1.throttlePercent, "throttle");
        addField(isValid(mcuMsg2.lastUpdate, mcuMsg2.timeoutMs, now), mcuMsg2.motorSpeed, "motor_speed");
        addField(isValid(cellVolt.lastUpdate, cellVolt.timeoutMs, now), cellVolt.maxCellVolt, "max_cell_voltage");
        addField(isValid(cellVolt.lastUpdate, cellVolt.timeoutMs, now), cellVolt.minCellVolt, "min_cell_voltage");

        unsigned long i2cAge = now - sensorData.lastScanTime;
        bool i2cFresh = (i2cAge <= DYNAMIC_SIGNAL_TIMEOUT_MS);
        for (const auto& pair : i2cValues) {
            double val = i2cFresh ? pair.second : 0.0;
            addField(true, val, pair.first);
        }

        if (gpsInitialized) {
            if (gpsData.location_valid) {
                addField(true, gpsData.latitude, "G-lat");
                addField(true, gpsData.longitude, "G-long");
            }
            if (gpsData.altitude_valid) {
                addField(true, gpsData.altitude, "G-alt");
            }
            if (gpsData.speed_valid) {
                addField(true, gpsData.speed_kmh, "gps_speed_kmh");
                addField(true, gpsData.speed_mps, "gps_speed_mps");
                addField(true, gpsData.speed_knots, "gps_speed_knots");
            }
            if (gpsData.course_valid) {
                addField(true, gpsData.course_deg, "gps_course");
            }
            if (gpsData.time_valid) {
                float utc_seconds = gpsData.hour_utc * 3600 + gpsData.minute_utc * 60 + gpsData.second_utc;
                addField(true, utc_seconds, "gps_utc_seconds");
                addField(true, gpsData.hour_utc, "gps_hour_utc");
                addField(true, gpsData.minute_utc, "gps_minute_utc");
                addField(true, gpsData.second_utc, "gps_second_utc");
            }
            if (gpsData.date_valid) {
                float date_numeric = gpsData.year * 10000 + gpsData.month * 100 + gpsData.day;
                addField(true, date_numeric, "gps_date");
            }
            if (gpsData.satellites_valid) {
                addField(true, gpsData.satellites, "gps_satellites");
            }
            if (gpsData.hdop_valid) {
                addField(true, gpsData.hdop, "gps_hdop");
            }
            if (compassData.valid) {
                addField(true, compassData.heading_deg, "compass_heading");
            }
            addField(true, gpsStats.maxSpeed_kmh, "gps_max_speed");
            addField(true, gpsStats.totalDistance_km, "gps_total_distance");
        }
    }

    if (dataAdded) payload += ",";
    else payload += " ";
    payload += "uptime=" + String(now / 1000.0, 2);

    if (payload.length() > mqttClient.getBufferSize()) {
        mqttClient.setBufferSize(payload.length() + 100);
    }

    if (mqttClient.publish(mqttTopic.c_str(), payload.c_str())) {
        lastMQTTSend = now;
        Serial.println("✅ MQTT data sent");
    } else {
        int state = mqttClient.state();
        Serial.printf("❌ MQTT publish failed, state=%d\n", state);
        if (!mqttClient.connected()) {
            mqttReady = false;
            espClient.stop();
        }
    }

    unsigned long duration = micros() - start;
    if (duration > 500000) {
        Serial.printf("⚠️ Long MQTT send: %lu µs\n", duration);
    }

    if (xSemaphoreTake(influxStatsMutex, portMAX_DELAY) == pdTRUE) {
        influxTotalTime += duration;
        influxCount++;
        xSemaphoreGive(influxStatsMutex);
    }
}

void sendInfluxDBData() {
    sendMQTTData();
}

// ================ Wi‑Fi station connection ================
void initWiFiStation() {
    Serial.print("Connecting to Wi-Fi station: ");
    Serial.println(wifiSSID);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ Wi-Fi station connected");
        Serial.print("Station IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n❌ Station connection failed (will retry later)");
    }
}

// ================ Initialize WiFi and MQTT ================
void initWiFi() {
    Serial.print("Starting WiFi Access Point... ");
    WiFi.setTxPower(WIFI_POWER_11dBm);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ap_ssid, ap_password);
    IPAddress ip = WiFi.softAPIP();
    wifiIP = ip.toString();
    wifiStatus = "AP Mode";
    Serial.println("✅ DONE");

    initWiFiStation();

    mqttClient.setServer(mqttBroker.c_str(), mqttPort);
    mqttClient.setKeepAlive(60);
    mqttClient.setBufferSize(2048);

    if (WiFi.status() == WL_CONNECTED) {
        connectMQTT();
    }
}

// ================ Web server setup ================
void startWebServer() {
    server.setContentLength(1024 * 1024);

    server.on("/api/dbc/upload", HTTP_POST, handleDBCUpload, handleDBCUploadFile);
    server.on("/api/dbc/parse", HTTP_GET, handleDBCParse);
    server.on("/api/dbc/save", HTTP_POST, handleDBCSave);
    server.on("/api/dbc/status", HTTP_GET, handleDBCStatus);
    server.on("/api/dbc/delete", HTTP_POST, handleDBCDelete);

    server.on("/api/i2c/config", HTTP_GET, handleI2CGetConfig);
    server.on("/api/i2c/config", HTTP_POST, handleI2CSaveConfig);
    server.on("/api/i2c/upload", HTTP_POST, handleI2CUpload, handleI2CUploadFile);
    server.on("/api/i2c/config/csv", HTTP_GET, handleI2CDownloadCSV);
    server.on("/i2c_config.html", HTTP_GET, handleI2CConfigPage);

    server.on("/dbc.html", []() {
        File file = SPIFFS.open("/dbc.html", FILE_READ);
        if (!file) {
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "text/html");
        file.close();
    });

    server.on("/", HTTP_GET, []() {
        server.sendHeader("Location", "/dbc.html");
        server.send(302, "text/plain", "");
    });

    server.onNotFound(handleNotFound);
    server.begin();

    createI2CConfigHTML();
}

void handleWebServer() {
    server.handleClient();
}

// ========== DBC Handlers ==========
void handleDBCUploadFile() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        size_t freeSpace = SPIFFS.totalBytes() - SPIFFS.usedBytes();
        if (upload.totalSize > freeSpace) {
            server.send(507, "text/plain", "Insufficient SPIFFS space");
            return;
        }
        String filename = "/dbc/upload.dbc";
        uploadFile = SPIFFS.open(filename, FILE_WRITE);
        Serial.printf("Upload start: %s\n", filename.c_str());
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) {
            size_t written = uploadFile.write(upload.buf, upload.currentSize);
            if (written != upload.currentSize) {
                uploadFile.close();
                SPIFFS.remove("/dbc/upload.dbc");
                server.send(500, "text/plain", "Write error");
                return;
            }
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) {
            uploadFile.close();
            Serial.printf("Upload end, size: %d\n", upload.totalSize);
            server.send(200, "text/plain", "Upload OK");
        } else {
            server.send(500, "text/plain", "File open failed");
        }
    }
}

void handleDBCUpload() {
   
}

void handleDBCParse() {
    
    if (ESP.getFreeHeap() < 60000) {
        server.send(503, "text/plain", "Insufficient memory – try again later");
        return;
    }

    std::vector<DBCMessage> messages;
    if (!parseDBCFile(SPIFFS, "/dbc/upload.dbc", messages)) {
        server.send(500, "text/plain", "Parse failed");
        return;
    }
    if (!saveDBCMessagesToJson(messages, "/dbc/messages.json")) {
        server.send(500, "text/plain", "Failed to save parsed data");
        return;
    }
    freeDBCMessages(messages);
    if (!SPIFFS.exists("/dbc/messages.json")) {
        server.send(500, "text/plain", "JSON file not found");
        return;
    }
    File jsonFile = SPIFFS.open("/dbc/messages.json", FILE_READ);
    if (!jsonFile) {
        server.send(500, "text/plain", "Cannot open JSON file");
        return;
    }
    server.streamFile(jsonFile, "application/json");
    jsonFile.close();
}

void handleDBCSave() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Body missing");
        return;
    }
    String body = server.arg("plain");
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, body);
    if (error) {
        server.send(400, "text/plain", "Invalid JSON");
        return;
    }

    std::vector<DBCMessage> messages;
    if (!loadDBCMessagesFromJson(messages, "/dbc/messages.json")) {
        server.send(500, "text/plain", "No parsed messages");
        return;
    }

    JsonArray selections = doc.as<JsonArray>();
    for (auto& msg : messages) {
        for (auto& sig : msg.signals) {
            sig.isSelected = false;
        }
    }
    for (JsonObject sel : selections) {
        uint32_t id = sel["id"];
        const char* sigName = sel["signal"];
        for (auto& msg : messages) {
            if (msg.id == id) {
                for (auto& sig : msg.signals) {
                    if (sig.name == sigName) {
                        sig.isSelected = true;
                        break;
                    }
                }
                break;
            }
        }
    }

    std::map<uint32_t, std::vector<DBCSignal>> canActiveMap;
    buildActiveMap(messages, canActiveMap);
    if (!saveSelectedSignals(SIGNAL_CONFIG_PATH, canActiveMap)) {
        server.send(500, "text/plain", "Failed to save CAN config");
        return;
    }

    setActiveSignals(canActiveMap);
    initDynamicValues(canActiveMap);
    setDynamicMode(true);
    Serial.println("Dynamic mode enabled via DBC save");
    server.send(200, "text/plain", "OK");
}

void handleDBCStatus() {
    DynamicJsonDocument doc(256);
    doc["dbc_loaded"] = SPIFFS.exists("/dbc/messages.json");
    doc["selected_count"] = 0;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleDBCDelete() {
    SPIFFS.remove("/dbc/upload.dbc");
    SPIFFS.remove("/dbc/messages.json");
    SPIFFS.remove(SIGNAL_CONFIG_PATH);
    setActiveSignals({});  
    setDynamicMode(false);
    server.send(200, "text/plain", "Deleted");
}

// ========== I2C Config Handlers (JSON) ==========
void handleI2CGetConfig() {
    DynamicJsonDocument doc(8192);
    JsonArray devices = doc.createNestedArray("devices");
    for (const auto& dev : i2cConfig.devices) {
        JsonObject d = devices.createNestedObject();
        d["type"] = dev.type;
        d["address"] = dev.address;
        JsonArray sigs = d.createNestedArray("signals");
        for (const auto& sig : dev.signals) {
            JsonObject s = sigs.createNestedObject();
            s["name"] = sig.name;
            s["enabled"] = sig.enabled;
            s["invert"] = sig.invert;
            s["factor"] = sig.factor;
            s["offset"] = sig.offset;
            s["unit"] = sig.unit;
            s["channel"] = sig.channel;
            s["bitMask"] = sig.bitMask;
            s["isMapped"] = sig.isMapped;
            if (sig.isMapped) {
                JsonObject mapObj = s.createNestedObject("mapping");
                for (const auto& pair : sig.valueMapping) {
                    mapObj[String(pair.first)] = pair.second;
                }
            }
        }
    }
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleI2CSaveConfig() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Missing body");
        return;
    }
    String body = server.arg("plain");
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, body);
    if (error) {
        server.send(400, "text/plain", "Invalid JSON");
        return;
    }

    I2CConfig newConfig;
    JsonArray devices = doc["devices"];
    for (JsonObject d : devices) {
        I2CDeviceConfig dev;
        dev.type = d["type"].as<String>();
        dev.address = d["address"];
        JsonArray sigs = d["signals"];
        for (JsonObject s : sigs) {
            I2CSignalConfig sig;
            sig.name = s["name"].as<String>();
            sig.enabled = s["enabled"];
            sig.invert = s["invert"] | false;
            sig.factor = s["factor"] | 1.0f;
            sig.offset = s["offset"] | 0.0f;
            sig.unit = s["unit"].as<String>();
            int8_t channel = s["channel"] | -1;
            sig.channel = channel;
            sig.bitMask = s["bitMask"] | 0;
            sig.isMapped = s["isMapped"] | false;
            if (sig.isMapped) {
                JsonObject mapObj = s["mapping"];
                for (JsonPair kv : mapObj) {
                    uint16_t key = atoi(kv.key().c_str());
                    String val = kv.value().as<String>();
                    sig.valueMapping[key] = val;
                }
            }
            dev.signals.push_back(sig);
        }
        newConfig.devices.push_back(dev);
    }

    if (saveI2CConfig(newConfig)) {
        
        i2cConfig = newConfig;
        server.send(200, "text/plain", "OK");
    } else {
        server.send(500, "text/plain", "Save failed");
    }
}

void handleI2CConfigPage() {
    
    if (!SPIFFS.exists("/i2c_config.html")) {
        createI2CConfigHTML();
    }
    File file = SPIFFS.open("/i2c_config.html", FILE_READ);
    if (!file) {
        server.send(404, "text/plain", "File not found");
        return;
    }
    server.streamFile(file, "text/html");
    file.close();
}

void createI2CConfigHTML() {
    File f = SPIFFS.open("/i2c_config.html", FILE_WRITE);
    if (!f) {
        Serial.println("Failed to create i2c_config.html");
        return;
    }

    f.print(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>I2C Sensor Configuration</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: 'Segoe UI', Arial, sans-serif; margin: 20px; background: #f4f7f9; }
        .container { max-width: 1200px; margin: auto; }
        .card { background: white; padding: 20px; margin: 15px 0; border-radius: 12px; box-shadow: 0 4px 8px rgba(0,0,0,0.05); }
        h1, h2 { color: #2c3e50; }
        .device { background: #eef2f5; margin: 10px 0; padding: 15px; border-radius: 8px; border-left: 5px solid #3498db; }
        .device-header { font-weight: bold; font-size: 1.1em; margin-bottom: 10px; display: flex; gap: 20px; align-items: center; }
        table { width: 100%; border-collapse: collapse; font-size: 0.9em; }
        th, td { padding: 8px 6px; border-bottom: 1px solid #ddd; text-align: left; white-space: nowrap; }
        th { background: #d5e6f2; }
        input[type=text], input[type=number] { width: 80px; padding: 4px; }
        input.sig-mapping { width: 150px; }
        .mapping-area { font-size: 0.9em; }
        button { background: #3498db; color: white; padding: 8px 14px; border: none; border-radius: 6px; cursor: pointer; margin-right: 8px; }
        button:hover { background: #2980b9; }
        button.secondary { background: #2ecc71; }
        button.remove { background: #e74c3c; }
        #status { margin-top: 10px; color: #27ae60; }
        .table-container { overflow-x: auto; }
    </style>
    <script>
        let config = { devices: [] };

        function loadConfig() {
            fetch('/api/i2c/config')
                .then(res => res.json())
                .then(data => {
                    config = data;
                    renderConfig();
                })
                .catch(err => console.error('Load error:', err));
        }

        function renderConfig() {
            const container = document.getElementById('devicesContainer');
            container.innerHTML = '';
            config.devices.forEach((dev, devIdx) => {
                const devDiv = document.createElement('div');
                devDiv.className = 'device';
                devDiv.innerHTML = `
                    <div class="device-header">
                        <span>${dev.type} @ 0x${dev.address.toString(16).toUpperCase()}</span>
                        <button class="remove" onclick="removeDevice(${devIdx})">Remove Device</button>
                    </div>
                    <div class="table-container">
                        <table>
                            <thead>
                                <tr>
                                    <th>Enable</th>
                                    <th>Signal Name</th>
                                    <th>Factor</th>
                                    <th>Offset</th>
                                    <th>Unit</th>
                                    <th>BitMask (hex)</th>
                                    <th>Invert</th>
                                    <th>Channel</th>
                                    <th>Mapping (raw:label)</th>
                                </tr>
                            </thead>
                            <tbody id="signals-${devIdx}"></tbody>
                        </table>
                    </div>
                    <button onclick="addSignal(${devIdx})">➕ Add Signal</button>
                `;
                container.appendChild(devDiv);

                const tbody = document.getElementById(`signals-${devIdx}`);
                dev.signals.forEach((sig, sigIdx) => {
                    const row = document.createElement('tr');
                    row.innerHTML = `
                        <td><input type="checkbox" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-enabled" ${sig.enabled ? 'checked' : ''}></td>
                        <td><input type="text" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-name" value="${sig.name}"></td>
                        <td><input type="number" step="0.01" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-factor" value="${sig.factor}"></td>
                        <td><input type="number" step="0.01" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-offset" value="${sig.offset}"></td>
                        <td><input type="text" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-unit" value="${sig.unit || ''}"></td>
                        <td><input type="text" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-bitmask" value="0x${sig.bitMask.toString(16)}"></td>
                        <td><input type="checkbox" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-invert" ${sig.invert ? 'checked' : ''}></td>
                        <td><input type="number" step="1" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-channel" value="${sig.channel !== undefined ? sig.channel : -1}"></td>
                        <td><input type="text" data-dev="${devIdx}" data-sig="${sigIdx}" class="sig-mapping" value="${mappingToString(sig.mapping)}"></td>
                    `;
                    tbody.appendChild(row);
                });
            });
        }

        function mappingToString(mapObj) {
            let parts = [];
            for (let key in mapObj) {
                parts.push(key + ':' + mapObj[key]);
            }
            return parts.join(',');
        }

        function parseMapping(str) {
            let map = {};
            str.split(',').forEach(pair => {
                let parts = pair.split(':');
                if (parts.length === 2) {
                    map[parts[0].trim()] = parts[1].trim();
                }
            });
            return map;
        }

        function collectConfig() {
            let newConfig = { devices: [] };
            config.devices.forEach((dev, devIdx) => {
                let newDev = {
                    type: dev.type,
                    address: dev.address,
                    signals: []
                };
                const rows = document.querySelectorAll(`#signals-${devIdx} tr`);
                rows.forEach((row, sigIdx) => {
                    let enabled = row.querySelector('.sig-enabled').checked;
                    let name = row.querySelector('.sig-name').value;
                    let factor = parseFloat(row.querySelector('.sig-factor').value);
                    let offset = parseFloat(row.querySelector('.sig-offset').value);
                    let unit = row.querySelector('.sig-unit').value;
                    let bitmaskStr = row.querySelector('.sig-bitmask').value;
                    let bitmask = parseInt(bitmaskStr, 16);
                    let invert = row.querySelector('.sig-invert').checked;
                    let channel = parseInt(row.querySelector('.sig-channel').value);
                    let mappingStr = row.querySelector('.sig-mapping').value;
                    let mapping = parseMapping(mappingStr);
                    let isMapped = Object.keys(mapping).length > 0;
                    newDev.signals.push({
                        name: name,
                        enabled: enabled,
                        factor: factor,
                        offset: offset,
                        unit: unit,
                        bitMask: bitmask,
                        invert: invert,
                        channel: channel,
                        isMapped: isMapped,
                        mapping: mapping
                    });
                });
                newConfig.devices.push(newDev);
            });
            return newConfig;
        }

        function addDevice() {
            let type = prompt("Enter device type (MCP9600, ADS1115, MCP23017, DS3231):");
            if (!type) return;
            let addrStr = prompt("Enter I2C address (hex, e.g., 0x60):");
            if (!addrStr) return;
            let addr = parseInt(addrStr, 16);
            config.devices.push({ type: type, address: addr, signals: [] });
            renderConfig();
        }

        function removeDevice(idx) {
            config.devices.splice(idx, 1);
            renderConfig();
        }

        function addSignal(devIdx) {
            let name = prompt("Signal name:");
            if (!name) return;
            config.devices[devIdx].signals.push({
                name: name,
                enabled: true,
                factor: 1.0,
                offset: 0.0,
                unit: '',
                bitMask: 0,
                invert: false,
                channel: -1,
                isMapped: false,
                mapping: {}
            });
            renderConfig();
        }

        function uploadCSV() {
            let file = document.getElementById('csvFile').files[0];
            if (!file) return;
            let formData = new FormData();
            formData.append('file', file);
            fetch('/api/i2c/upload', { method: 'POST', body: formData })
                .then(res => res.text())
                .then(msg => {
                    document.getElementById('status').innerText = msg;
                    loadConfig();
                })
                .catch(err => console.error('Upload error:', err));
        }

        function downloadCSV() {
            window.location.href = '/api/i2c/config/csv';
        }

        function saveConfig() {
            let newConfig = collectConfig();
            fetch('/api/i2c/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(newConfig)
            })
            .then(res => res.text())
            .then(msg => {
                document.getElementById('status').innerText = 'Saved: ' + msg;
            })
            .catch(err => console.error('Save error:', err));
        }

        window.onload = loadConfig;
    </script>
</head>
<body>
<div class="container">
    <h1>🔧 I2C Sensor Configuration</h1>
    <div class="card">
        <h2>Upload CSV</h2>
        <input type="file" id="csvFile" accept=".csv">
        <button onclick="uploadCSV()">⬆ Upload & Replace</button>
        <button onclick="downloadCSV()">⬇ Download CSV</button>
        <div id="status"></div>
    </div>
    <div class="card">
        <h2>Devices & Signals</h2>
        <button onclick="addDevice()">➕ Add Device</button>
        <div id="devicesContainer"></div>
        <button onclick="saveConfig()">💾 Save to ESP32</button>
    </div>
</div>
</body>
</html>
)rawliteral");

    f.close();
}

// ========== NEW: I2C CSV Upload/Download ==========
static File i2cUploadFile;

void handleI2CUploadFile() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        String filename = "/config/i2c_upload.csv";
        i2cUploadFile = SPIFFS.open(filename, FILE_WRITE);
        Serial.printf("I2C CSV upload start: %s\n", filename.c_str());
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (i2cUploadFile) {
            size_t written = i2cUploadFile.write(upload.buf, upload.currentSize);
            if (written != upload.currentSize) {
                i2cUploadFile.close();
                SPIFFS.remove("/config/i2c_upload.csv");
                server.send(500, "text/plain", "Write error");
                return;
            }
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (i2cUploadFile) {
            i2cUploadFile.close();
            Serial.printf("I2C CSV upload end, size: %d\n", upload.totalSize);

            File f = SPIFFS.open("/config/i2c_upload.csv", FILE_READ);
            String csv = f.readString();
            f.close();

            I2CConfig newConfig;
            if (parseI2CConfigCSV(csv, newConfig)) {
                i2cConfig = newConfig;
                saveI2CConfig(i2cConfig);
                server.send(200, "text/plain", "CSV processed and saved");
            } else {
                server.send(500, "text/plain", "CSV parse failed");
            }
        } else {
            server.send(500, "text/plain", "Upload failed");
        }
    }
}

void handleI2CUpload() {
    
}

void handleI2CDownloadCSV() {
    String csv = i2cConfigToCSV(i2cConfig);
    server.send(200, "text/csv", csv);
}

// ========== 404 handler ==========
void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

// ========== Create dbc.html if missing ==========
void createDBCFileIfNeeded() {
    if (!SPIFFS.exists("/dbc.html")) {
        Serial.println("Creating dbc.html");
        File f = SPIFFS.open("/dbc.html", FILE_WRITE);
        f.print(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>DBC Signal Selector</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: 'Segoe UI', Arial, sans-serif; margin: 20px; background: #f4f7f9; }
        .container { max-width: 1000px; margin: auto; }
        .card { background: white; padding: 20px; margin: 15px 0; border-radius: 12px; box-shadow: 0 4px 8px rgba(0,0,0,0.05); }
        h1, h2, h3 { color: #2c3e50; margin-top: 0; }
        .message { background: #f9f9fc; border-left: 4px solid #3498db; margin-bottom: 20px; padding: 12px; border-radius: 8px; }
        .msg-header { display: flex; align-items: center; gap: 15px; font-weight: bold; font-size: 1.1em; margin-bottom: 10px; background: #eaf2f8; padding: 8px 12px; border-radius: 6px; }
        .msg-header input[type=checkbox] { transform: scale(1.2); margin-right: 5px; }
        table { width: 100%; border-collapse: collapse; font-size: 0.95em; }
        th { text-align: left; background: #d5e6f2; padding: 8px 6px; font-weight: 600; }
        td { padding: 8px 6px; border-bottom: 1px solid #ddd; }
        tr:hover { background: #f1f9ff; }
        .signal-checkbox { transform: scale(1.2); margin-right: 5px; }
        .bullet { color: #2c3e50; margin-right: 6px; font-size: 1.2em; }
        .unit, .receiver { color: #555; font-style: italic; }
        button { background: #3498db; color: white; padding: 10px 18px; border: none; border-radius: 6px; cursor: pointer; font-size: 1em; margin-right: 8px; transition: background 0.2s; }
        button:hover { background: #2980b9; }
        button.secondary { background: #2ecc71; }
        button.secondary:hover { background: #27ae60; }
        button.primary { background: #27ae60; }
        button.primary:hover { background: #2ecc71; }
        #status { color: #27ae60; font-weight: 500; margin-top: 10px; }
        #selectedOutput { background: #edf2f7; padding: 12px; border-radius: 8px; font-family: 'Courier New', monospace; white-space: pre-wrap; max-height: 250px; overflow-y: auto; border: 1px solid #ccc; margin-top: 10px; }
        .file-input { margin: 15px 0; }
    </style>
    <script defer>
        console.log("Script loaded");
        function uploadDBC() {
            let file = document.getElementById('dbcFile').files[0];
            if (!file) return;
            let formData = new FormData();
            formData.append('file', file);
            fetch('/api/dbc/upload', { method: 'POST', body: formData })
                .then(res => res.text())
                .then(msg => {
                    document.getElementById('status').innerText = msg;
                    parseDBC();
                })
                .catch(err => console.error('Upload error:', err));
        }
        function parseDBC() {
            fetch('/api/dbc/parse')
                .then(res => res.json())
                .then(data => {
                    renderTree(data);
                    document.getElementById('signalsCard').style.display = 'block';
                })
                .catch(err => console.error('Parse error:', err));
        }
        function renderTree(messages) {
            let html = '';
            messages.forEach(msg => {
                const msgId = msg.id;
                html += `<div class="message" data-msg-id="${msgId}">`;
                html += `<div class="msg-header">`;
                html += `<input type="checkbox" class="select-all" data-msg-id="${msgId}" onchange="toggleAll(this, ${msgId})">`;
                html += `<span><strong>${msg.hexId} (${msg.id}) – ${msg.name}</strong>  (${msg.dlc} bytes, tx: ${msg.transmitter})</span>`;
                html += `</div>`;
                html += `<table>`;
                html += `<thead><tr><th style="width:30px">✔</th><th>Signal</th><th>Bit</th><th>Type</th><th>Scale/Offset</th><th>Unit</th><th>Receiver</th></tr></thead>`;
                html += `<tbody>`;
                msg.signals.forEach(sig => {
                    const sigId = `sig-${msgId}-${sig.name.replace(/\s/g, '_')}`;
                    const bitDesc = `${sig.startBit}|${sig.length}@${sig.intel ? 'Intel' : 'Motorola'}${sig.signed ? '-' : ''}`;
                    const scaleOffset = `(${sig.scale},${sig.offset})`;
                    html += `<tr>`;
                    html += `<td style="text-align:center"><input type="checkbox" class="signal-checkbox" id="${sigId}" data-msg-id="${msgId}" data-sig-name="${sig.name}" onchange="updateSelectAll('${msgId}')"></td>`;
                    html += `<td><label for="${sigId}"><span class="bullet">•</span>${sig.name}</label></td>`;
                    html += `<td>${bitDesc}</td>`;
                    html += `<td>${sig.signed ? 'signed' : 'unsigned'}</td>`;
                    html += `<td>${scaleOffset}</td>`;
                    html += `<td class="unit">${sig.unit || '—'}</td>`;
                    html += `<td class="receiver">${sig.receiver || '—'}</td>`;
                    html += `</tr>`;
                });
                html += `</tbody></table>`;
                html += `</div>`;
            });
            document.getElementById('signalsTree').innerHTML = html;
        }
        function toggleAll(selectAllCheckbox, msgId) {
            const checkboxes = document.querySelectorAll(`.signal-checkbox[data-msg-id="${msgId}"]`);
            checkboxes.forEach(cb => cb.checked = selectAllCheckbox.checked);
        }
        function updateSelectAll(msgId) {
            const checkboxes = document.querySelectorAll(`.signal-checkbox[data-msg-id="${msgId}"]`);
            const selectAll = document.querySelector(`.select-all[data-msg-id="${msgId}"]`);
            if (!selectAll) return;
            const allChecked = Array.from(checkboxes).every(cb => cb.checked);
            selectAll.checked = allChecked;
            selectAll.indeterminate = !allChecked && Array.from(checkboxes).some(cb => cb.checked);
        }
        function showSelected() {
            const selected = [];
            document.querySelectorAll('.signal-checkbox:checked').forEach(cb => {
                const msgId = cb.dataset.msgId;
                const sigName = cb.dataset.sigName;
                const msgHeader = document.querySelector(`.message[data-msg-id="${msgId}"] .msg-header span`);
                let msgName = '';
                if (msgHeader) {
                    const parts = msgHeader.innerText.split('–');
                    if (parts.length > 1) msgName = parts[1].trim().split(' ')[0];
                }
                selected.push(`Message: ${msgName} (ID ${msgId})  →  Signal: ${sigName}`);
            });
            const outputDiv = document.getElementById('selectedOutput');
            if (selected.length === 0) outputDiv.innerText = 'No signals selected.';
            else outputDiv.innerText = selected.join('\n');
        }
        function clearSelections() {
            document.querySelectorAll('.signal-checkbox').forEach(cb => cb.checked = false);
            document.querySelectorAll('.select-all').forEach(cb => { cb.checked = false; cb.indeterminate = false; });
            document.getElementById('selectedOutput').innerText = '';
        }
        function saveSelection() {
            let selected = [];
            document.querySelectorAll('.signal-checkbox:checked').forEach(cb => {
                const msgId = cb.dataset.msgId;
                const sigName = cb.dataset.sigName;
                selected.push({ id: parseInt(msgId), signal: sigName });
            });
            fetch('/api/dbc/save', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(selected)
            })
            .then(res => res.text())
            .then(msg => {
                alert('Saved: ' + msg);
            })
            .catch(err => console.error('Save error:', err));
        }
    </script>
</head>
<body>
<div class="container">
    <h1>📊 DBC Signal Selector</h1>
    <div class="card">
        <h2>Upload DBC File</h2>
        <div class="file-input">
            <input type="file" id="dbcFile" accept=".dbc">
        </div>
        <button onclick="uploadDBC()">⬆ Upload & Parse</button>
        <div id="status"></div>
    </div>
    <div class="card" id="signalsCard" style="display:none;">
        <h2>📋 Messages & Signals</h2>
        <div style="margin-bottom:15px;">
            <button class="secondary" onclick="showSelected()">✓ Show Selected Signals</button>
            <button onclick="clearSelections()">🗑 Clear Selections</button>
            <button class="primary" onclick="saveSelection()">💾 Save Configuration</button>
        </div>
        <div id="signalsTree"></div>
        <h3>📌 Selected Signals:</h3>
        <div id="selectedOutput"></div>
    </div>
</div>
</body>
</html>
)rawliteral");
        f.close();
        
    }
}