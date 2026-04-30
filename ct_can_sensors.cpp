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

// ================ NMT Command ================
void sendNMTStartCommand() {
    twai_message_t message;
    message.identifier = CAN_ID_NMT;
    message.extd = 0;
    message.data_length_code = 2;
    message.data[0] = NMT_START_NODE;
    message.data[1] = 0x00;  // Broadcast to all nodes
    
    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        Serial.println("[CT-CAN] NMT Start command sent to all nodes");
    } else {
        Serial.println("[CT-CAN] Failed to send NMT Start command");
    }
}

// ================ Process CAN Messages ================
void processCTCANMessage(const twai_message_t& msg) {
    unsigned long now = millis();
    uint32_t id = msg.identifier;
    
    // Flow Sensor (0x181) - Node 0x01
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
        
        // Removed: filteredMessageCount++, lastFilteredTime, session start logic
    }
    
    // Pressure Sensor (0x182) - Node 0x02
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
        
        // Removed: filteredMessageCount++, lastFilteredTime, session start logic
    }
    
    // Temperature Sensor (0x183) - Node 0x03
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
        
        // Removed: filteredMessageCount++, lastFilteredTime, session start logic
    }
}

// ================ Initialize CT Sensors ================
void initCTSensors() {
    Serial.println("[CT-CAN] Initializing CT Sensors...");
    
    // Initialize data structures
    memset(&ctFlow, 0, sizeof(CTFlowData));
    memset(&ctPressure, 0, sizeof(CTPressureData));
    memset(&ctTemp, 0, sizeof(CTTemperatureData));
    
    ctFlow.timeoutMs = 1000;
    ctPressure.timeoutMs = 1000;
    ctTemp.timeoutMs = 2000;
    
    // Send NMT start command
    sendNMTStartCommand();
    
    ctSensorsInitialized = true;
    Serial.println("[CT-CAN] CT Sensors initialized");
}

// ================ Update and Check Timeouts ================
void updateCTSensors() {
    // This function can be called periodically to check timeouts
    // Currently handled in processCTCANMessage
}