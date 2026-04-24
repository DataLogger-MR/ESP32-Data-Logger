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

// Set to false if your DBC uses LSB start bits for Motorola signals (as in your matrix)
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
        // Intel: startBit is LSB, bits in increasing order
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
            // Standard Motorola: startBit is MSB, bits decreasing within byte, then next byte's MSB
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
            // Alternative: startBit is LSB, bytes are big-endian
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

    // Handle signed (two's complement)
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
            lastDynamicUpdateTime[sig.name] = millis();   // Record the timestamp
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

// ================ BUILD DYNAMIC CSV HEADER (includes I2C signals) ================
String buildDynamicCSVHeader() {
    String header = "Timestamp_ms";
    // DBC signals
    for (const auto& pair : activeSignals) {
        for (const auto& sig : pair.second) {
            header += "," + sig.name;
        }
    }
    // Hardcoded thermocouples with short names
    header += ",TC1";
    header += ",TC2";
    header += ",TC3";
    header += ",TC4";
    header += ",TC5";
    header += ",TC6";
    header += ",TC7";
    header += ",TC8";
    header += ",TC9";
    header += ",TC10";
    header += ",TC11";
    header += ",TC12";

    // I2C signals from config (skip MCP9600, which is already handled above)
    for (const auto& dev : i2cConfig.devices) {
        if (dev.type == "MCP9600") continue;   // skip to avoid duplicates
        for (const auto& sig : dev.signals) {
            if (sig.enabled) {
                header += "," + sig.name;
            }
        }
    }

    header += ",Speed_RPM";
    // GPS columns - ONLY 4 PARAMETERS
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