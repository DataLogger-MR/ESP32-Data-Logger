#include "dynamic_decoder.h"
#include "globals.h"
#include "session_manager.h"
#include "file_manager.h"
#include "gps_globals.h"
#include "i2c_sensors.h"
#include "i2c_config.h"

std::map<uint32_t, std::vector<DBCSignal>> activeSignals;
extern SemaphoreHandle_t dataMutex;
extern std::map<String, double> lastDynamicValues;
std::map<String, unsigned long> lastDynamicUpdateTime;

bool motorolaStartBitIsMSB = true;

void initDynamicDecoder() {
    activeSignals.clear();
}

void setActiveSignals(const std::map<uint32_t, std::vector<DBCSignal>>& newMap) {
    activeSignals = newMap;
    Serial.printf("setActiveSignals: %d messages, total signals: ", newMap.size());
    size_t total = 0;
    for (auto& p : newMap) total += p.second.size();
    Serial.println(total);
}

double extractSignalValue(const uint8_t* data, uint8_t startBit, uint8_t length, bool intel, bool isSigned) {
    uint64_t raw = 0;
    if (intel) {
        
        int bytePos = startBit / 8;
        int bitInByte = startBit % 8;
        for (int i = 0; i < length; i++) {
            if (data[bytePos] & (1 << bitInByte)) {
                raw |= (1ULL << i);
            }
            if (++bitInByte == 8) {
                bytePos++;
                bitInByte = 0;
            }
        }
    } else {
        if (motorolaStartBitIsMSB) {
           
            int bytePos = startBit / 8;
            int bitInByte = startBit % 8;
            for (int i = 0; i < length; i++) {
                if (data[bytePos] & (1 << bitInByte)) {
                    raw |= (1ULL << (length - 1 - i));
                }
                if (--bitInByte < 0) {
                    bytePos++;
                    bitInByte = 7;
                }
            }
        } else {
           
            uint64_t rawLE = 0;
            int bytePos = startBit / 8;
            int bitInByte = startBit % 8;
            for (int i = 0; i < length; i++) {
                if (data[bytePos] & (1 << bitInByte)) {
                    rawLE |= (1ULL << i);
                }
                if (++bitInByte == 8) {
                    bytePos++;
                    bitInByte = 0;
                }
            }
            int numBytes = (length + 7) / 8;
            for (int i = 0; i < numBytes; i++) {
                uint8_t byteVal = (rawLE >> (i * 8)) & 0xFF;
                raw |= (uint64_t)byteVal << ((numBytes - 1 - i) * 8);
            }
        }
    }

    if (isSigned && (raw & (1ULL << (length - 1)))) {
        raw = raw - (1ULL << length);
    }
    return (double)raw;
}

void decodeDynamic(const twai_message_t& msg, std::map<String, double>& outValues) {
    auto it = activeSignals.find(msg.identifier);
    if (it == activeSignals.end()) return;
    
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        for (const auto& sig : it->second) {
            double raw = extractSignalValue(msg.data, sig.startBit, sig.length, sig.isIntel, sig.isSigned);
            double phys = raw * sig.scale + sig.offset;
            outValues[sig.name] = phys;
            lastDynamicUpdateTime[sig.name] = millis();  
        }
        xSemaphoreGive(dataMutex);
    }
    
    filteredMessageCount++;
    lastFilteredTime = millis();
    if (sessionState == SESSION_STATE_WAITING) {
        startNewSession();
        createNewLogFile();
        sessionState = SESSION_STATE_ACTIVE;
    }
}

// ================ CSV HEADER  ================
String buildDynamicCSVHeader() {
    String header = "Timestamp_ms";
    
    for (const auto& pair : activeSignals) {
        for (const auto& sig : pair.second) {
            header += "," + sig.name;
        }
    }
    
    header += ",TC1,TC2,TC3,TC4,TC5,TC6,TC7,TC8,TC9,TC10,TC11,TC12";

    for (const auto& dev : i2cConfig.devices) {
        if (dev.type == "MCP9600") continue;
        for (const auto& sig : dev.signals) {
            if (sig.enabled) {
                header += "," + sig.name;
            }
        }
    }

    header += ",Speed_RPM";
    header += ",Flow_LPM,Pressure_Bar,Temp_Celsius";
    header += ",Latitude,Longitude,Altitude_m,Speed_kmh";
    return header;
}

String buildDynamicCSVRow(unsigned long timestamp, const std::map<String, double>& values) {
    String row = String(timestamp);
    for (const auto& pair : activeSignals) {
        for (const auto& sig : pair.second) {
            auto it = values.find(sig.name);
            if (it != values.end()) {
                row += "," + String(it->second, 3);
            } else {
                row += ",";
            }
        }
    }
    return row;
}

void initDynamicValues(const std::map<uint32_t, std::vector<DBCSignal>>& activeMap) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        lastDynamicValues.clear();
        for (const auto& msgPair : activeMap) {
            for (const auto& sig : msgPair.second) {
                lastDynamicValues[sig.name] = 0.0;
            }
        }
        xSemaphoreGive(dataMutex);
    }
}