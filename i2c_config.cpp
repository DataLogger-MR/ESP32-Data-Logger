#include "i2c_config.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>

I2CConfig i2cConfig;
bool i2cConfigLoaded = false;

// ------------------------------------------------------------------
// Save config to SPIFFS as JSON
bool saveI2CConfig(const I2CConfig& config) {
    DynamicJsonDocument doc(8192);
    JsonArray devices = doc.createNestedArray("devices");

    for (const auto& dev : config.devices) {
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
            s["bitMask"] = sig.bitMask;
            s["isMapped"] = sig.isMapped;
            s["channel"] = sig.channel;   // <-- new field
            if (sig.isMapped) {
                JsonObject mapObj = s.createNestedObject("mapping");
                for (const auto& pair : sig.valueMapping) {
                    mapObj[String(pair.first)] = pair.second;
                }
            }
        }
    }

    File file = SPIFFS.open("/config/i2c_config.json", FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open I2C config for writing");
        return false;
    }
    serializeJson(doc, file);
    file.close();
    Serial.println("I2C config saved.");
    return true;
}

void loadI2CConfig() {
    if (!SPIFFS.exists("/config/i2c_config.json")) {
        Serial.println("I2C config not found, creating empty config.");
        i2cConfig.devices.clear();
        i2cConfigLoaded = true;
        saveI2CConfig(i2cConfig);
        return;
    }

    File file = SPIFFS.open("/config/i2c_config.json", FILE_READ);
    if (!file) {
        Serial.println("Failed to open I2C config, using empty.");
        i2cConfig.devices.clear();
        i2cConfigLoaded = true;
        return;
    }

    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.printf("I2C config parse error: %s\n", error.c_str());
        i2cConfig.devices.clear();
        i2cConfigLoaded = true;
        return;
    }

    i2cConfig.devices.clear();
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
            sig.bitMask = s["bitMask"] | 0;
            sig.isMapped = s["isMapped"] | false;
            sig.channel = s["channel"] | -1;   // <-- new, default -1
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
        i2cConfig.devices.push_back(dev);
    }
    i2cConfigLoaded = true;
    Serial.println("I2C config loaded.");
}

// ------------------------------------------------------------------
// CSV parsing (simple, assumes no quoted commas)
bool parseI2CConfigCSV(const String& csv, I2CConfig& config) {
    config.devices.clear();
    std::map<String, I2CDeviceConfig*> deviceMap; // key = type+":"+address

    int lineStart = 0;
    int lineEnd;
    bool firstLine = true;
    while ((lineEnd = csv.indexOf('\n', lineStart)) != -1) {
        String line = csv.substring(lineStart, lineEnd);
        line.trim();
        lineStart = lineEnd + 1;
        if (line.length() == 0) continue;
        if (firstLine) { firstLine = false; continue; } // skip header

        std::vector<String> fields;
        int fieldStart = 0, fieldEnd;
        while ((fieldEnd = line.indexOf(',', fieldStart)) != -1) {
            fields.push_back(line.substring(fieldStart, fieldEnd));
            fieldStart = fieldEnd + 1;
        }
        fields.push_back(line.substring(fieldStart));
        // Expect 11 columns: DeviceType,Address,SignalName,Enabled,Factor,Offset,Unit,BitMask,Invert,Channel,Mapping
        if (fields.size() < 11) continue;

        String devType = fields[0];
        uint8_t addr = (uint8_t)strtol(fields[1].c_str(), NULL, 16);
        String sigName = fields[2];
        bool enabled = fields[3].toInt() != 0;
        float factor = fields[4].toFloat();
        float offset = fields[5].toFloat();
        String unit = fields[6];
        const char* bitStr = fields[7].c_str();
        uint16_t bitMask;
        if (bitStr[0] == '0' && (bitStr[1] == 'x' || bitStr[1] == 'X')) {
            bitMask = (uint16_t)strtol(bitStr, NULL, 16);
        } else {
            bitMask = (uint16_t)strtol(bitStr, NULL, 10);
        }
        bool invert = fields[8].toInt() != 0;
        int8_t channel = fields[9].toInt();            // <-- new
        String mappingStr = fields[10];

        String devKey = devType + ":" + String(addr);
        I2CDeviceConfig* dev = nullptr;
        auto it = deviceMap.find(devKey);
        if (it == deviceMap.end()) {
            I2CDeviceConfig newDev;
            newDev.type = devType;
            newDev.address = addr;
            config.devices.push_back(newDev);
            dev = &config.devices.back();
            deviceMap[devKey] = dev;
        } else {
            dev = it->second;
        }

        std::map<uint16_t, String> valueMapping;
        bool isMapped = false;
        if (mappingStr.length() > 0) {
            int mapStart = 0, mapEnd;
            while ((mapEnd = mappingStr.indexOf(',', mapStart)) != -1) {
                String pair = mappingStr.substring(mapStart, mapEnd);
                int colon = pair.indexOf(':');
                if (colon > 0) {
                    uint16_t key = (uint16_t)pair.substring(0, colon).toInt();
                    String val = pair.substring(colon + 1);
                    valueMapping[key] = val;
                    isMapped = true;
                }
                mapStart = mapEnd + 1;
            }
            String pair = mappingStr.substring(mapStart);
            int colon = pair.indexOf(':');
            if (colon > 0) {
                uint16_t key = (uint16_t)pair.substring(0, colon).toInt();
                String val = pair.substring(colon + 1);
                valueMapping[key] = val;
                isMapped = true;
            }
        }

        I2CSignalConfig sig;
        sig.name = sigName;
        sig.enabled = enabled;
        sig.factor = factor;
        sig.offset = offset;
        sig.unit = unit;
        sig.bitMask = bitMask;
        sig.isMapped = isMapped;
        sig.invert = invert;
        sig.channel = channel;          // <-- new
        sig.valueMapping = valueMapping;
        dev->signals.push_back(sig);
    }
    return true;
}

String i2cConfigToCSV(const I2CConfig& config) {
    String csv = "DeviceType,Address,SignalName,Enabled,Factor,Offset,Unit,BitMask,Invert,Channel,Mapping\n";
    for (const auto& dev : config.devices) {
        for (const auto& sig : dev.signals) {
            csv += dev.type + ",";
            csv += "0x" + String(dev.address, HEX) + ",";
            csv += sig.name + ",";
            csv += String(sig.enabled ? "1" : "0") + ",";
            csv += String(sig.factor) + ",";
            csv += String(sig.offset) + ",";
            csv += sig.unit + ",";
            csv += "0x" + String(sig.bitMask, HEX) + ",";
            csv += String(sig.invert ? "1" : "0") + ",";
            csv += String(sig.channel) + ",";   // <-- new column
            if (sig.isMapped) {
                bool first = true;
                for (const auto& pair : sig.valueMapping) {
                    if (!first) csv += ",";
                    first = false;
                    csv += String(pair.first) + ":" + pair.second;
                }
            }
            csv += "\n";
        }
    }
    return csv;
}