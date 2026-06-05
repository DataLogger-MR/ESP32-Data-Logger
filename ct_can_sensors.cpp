#include "ct_can_sensors.h"
#include "globals.h"
#include "session_manager.h"
#include "file_manager.h"
#include "utils.h"

// ================ Global Definitions ================
CTFlowData ctFlow = {0, 0, 0, false, 0, 1000};
CTPressureData ctPressure = {0, 0, 0, false, 0, 1000};
CTTemperatureData ctTemp = {0, 0, 0, false, 0, 1000};
bool ctSensorsInitialized = false;

// ================ NMT State Variables ================
unsigned long lastHeartbeatCheck = 0;
unsigned long lastNMTStartAttempt = 0;
bool nmtStartAttempted = false;
bool allSensorsStarted = false;

// ================ Helper Functions ================
float bytesToFloatCT(const uint8_t* bytes) {
    uint32_t combined = (uint32_t)bytes[0] | 
                        (uint32_t)bytes[1] << 8 | 
                        (uint32_t)bytes[2] << 16 | 
                        (uint32_t)bytes[3] << 24;
    float result;
    memcpy(&result, &combined, sizeof(result));
    return result;
}

String getCTStatusString(uint8_t status) {
    switch(status) {
        case CT_STATUS_VALID: return "VALID";
        case CT_STATUS_BROKEN_WIRE: return "BROKEN_WIRE";
        case CT_STATUS_RANGE_HIGH: return "RANGE_HIGH";
        case CT_STATUS_RANGE_LOW: return "RANGE_LOW";
        default: return "UNKNOWN";
    }
}

bool isCTDataValid(unsigned long lastUpdate, unsigned long timeoutMs, unsigned long currentTime) {
    if (lastUpdate == 0) return false;
    if (timeoutMs == 0) return true;
    return (currentTime - lastUpdate) <= timeoutMs;
}

// ================ NMT Command Functions ================
void sendNMTCommand(uint8_t nodeId, uint8_t command) {
    twai_message_t message;
    message.identifier = CAN_ID_NMT;
    message.extd = 0;
    message.data_length_code = 2;
    message.data[0] = command;
    message.data[1] = nodeId;
    
    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        Serial.printf("[CT-CAN] NMT command %02X sent to node %d\n", command, nodeId);
    } else {
        Serial.printf("[CT-CAN] Failed to send NMT command to node %d\n", nodeId);
    }
}

void sendNMTStartCommandToNode(uint8_t nodeId) {
    sendNMTCommand(nodeId, NMT_START_NODE);
}

void sendNMTStartCommand() {
    sendNMTCommand(0x00, NMT_START_NODE);
    allSensorsStarted = true;
}

void forceStartAllCTNodes() {
    Serial.println("[CT-CAN] Force starting all CT nodes...");
    sendNMTStartCommand();
    lastNMTStartAttempt = millis();
    
    for (uint8_t nodeId = 1; nodeId <= 127; nodeId++) {
        sendNMTStartCommandToNode(nodeId);
        delay(5);
    }
}

bool isAnyCTSensorValid() {
    unsigned long now = millis();
    return (ctFlow.valid && (now - ctFlow.lastUpdate) <= ctFlow.timeoutMs) ||
           (ctPressure.valid && (now - ctPressure.lastUpdate) <= ctPressure.timeoutMs) ||
           (ctTemp.valid && (now - ctTemp.lastUpdate) <= ctTemp.timeoutMs);
}

// ================ Process CAN Messages ================
void processCTCANMessage(const twai_message_t& msg) {
    unsigned long now = millis();
    uint32_t id = msg.identifier;
    
    if (id == CAN_ID_FLOW && msg.data_length_code >= 5) {
        ctFlow.flow_lpm = bytesToFloatCT(msg.data);
        ctFlow.flow_gpm = ctFlow.flow_lpm * 0.264172f;
        ctFlow.status = msg.data[4];
        ctFlow.valid = true;
        ctFlow.lastUpdate = now;
        
        #if ENABLE_SERIAL_DEBUG
            Serial.printf("[CT-FLOW] %.2f LPM (%.2f GPM) | Status: %s\n", 
                         ctFlow.flow_lpm, ctFlow.flow_gpm, 
                         getCTStatusString(ctFlow.status).c_str());
        #endif
    }
    else if (id == CAN_ID_PRESSURE && msg.data_length_code >= 5) {
        ctPressure.pressure_bar = bytesToFloatCT(msg.data);
        ctPressure.pressure_psi = ctPressure.pressure_bar * 14.5038f;
        ctPressure.status = msg.data[4];
        ctPressure.valid = true;
        ctPressure.lastUpdate = now;
        
        #if ENABLE_SERIAL_DEBUG
            Serial.printf("[CT-PRESSURE] %.2f Bar (%.2f PSI) | Status: %s\n", 
                         ctPressure.pressure_bar, ctPressure.pressure_psi,
                         getCTStatusString(ctPressure.status).c_str());
        #endif
    }
    else if (id == CAN_ID_TEMP && msg.data_length_code >= 5) {
        ctTemp.temp_celsius = bytesToFloatCT(msg.data);
        ctTemp.temp_fahrenheit = ctTemp.temp_celsius * 1.8f + 32;
        ctTemp.status = msg.data[4];
        ctTemp.valid = true;
        ctTemp.lastUpdate = now;
        
        #if ENABLE_SERIAL_DEBUG
            Serial.printf("[CT-TEMP] %.2f °C (%.2f °F) | Status: %s\n", 
                         ctTemp.temp_celsius, ctTemp.temp_fahrenheit,
                         getCTStatusString(ctTemp.status).c_str());
        #endif
    }
    else if ((id & 0x700) == 0x700) {  
        uint8_t nodeState = msg.data[0];
        uint8_t nodeId = id & 0x7F;
        
        if (nodeState == 0x04 || nodeState == 0x7F) {  
            Serial.printf("[CT-CAN] Node %d in state 0x%02X, sending NMT start\n", nodeId, nodeState);
            sendNMTStartCommandToNode(nodeId);
        }
    }
}

// ================ Liveness Check ================
void checkCTSensorsLiveness() {
    unsigned long now = millis();
    
    if (!ctSensorsInitialized) return;
    
    if (now - lastHeartbeatCheck < 5000) return;
    lastHeartbeatCheck = now;
    
    bool everValid = ctFlow.valid || ctPressure.valid || ctTemp.valid;
    
    if (!everValid) {

        if (!nmtStartAttempted || (now - lastNMTStartAttempt > 10000)) {
            Serial.println("[CT-CAN] No sensor data ever received, sending NMT start...");
            forceStartAllCTNodes();
            nmtStartAttempted = true;
            lastNMTStartAttempt = now;
        }
    } 
    else if (!isAnyCTSensorValid()) {
  
        if (now - lastNMTStartAttempt > 10000) {
            Serial.println("[CT-CAN] All CT sensors stale, re-sending NMT start...");
            forceStartAllCTNodes();
            lastNMTStartAttempt = now;
        }
    }
}

// ================ Initialize CT Sensors ================
void initCTSensors() {
    Serial.println("[CT-CAN] Initializing CT Sensors...");
    
    memset(&ctFlow, 0, sizeof(CTFlowData));
    memset(&ctPressure, 0, sizeof(CTPressureData));
    memset(&ctTemp, 0, sizeof(CTTemperatureData));
    
    ctFlow.timeoutMs = 1000;
    ctPressure.timeoutMs = 1000;
    ctTemp.timeoutMs = 2000;
    
    lastHeartbeatCheck = 0;
    lastNMTStartAttempt = 0;
    nmtStartAttempted = false;
    allSensorsStarted = false;
    
    forceStartAllCTNodes();
    nmtStartAttempted = true;
    lastNMTStartAttempt = millis();
    
    ctSensorsInitialized = true;
    Serial.println("[CT-CAN] CT Sensors initialized");
}

// ================ Update and Check Timeouts ================
void updateCTSensors() {
    checkCTSensorsLiveness();
}