#include "signal_selector.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "esp_task_wdt.h"
#include "dbc_parser.h"
#include "config.h"
#include <set>          // <-- add this line

bool loadSelectedSignals(const char* path, std::map<uint32_t, std::vector<DBCSignal>>& activeMap) {
    File file = SPIFFS.open(path, FILE_READ);
    if (!file) return false;
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) return false;
    
    JsonArray arr = doc.as<JsonArray>();
    for (JsonObject item : arr) {
        uint32_t id = item["id"];
        const char* sigName = item["signal"];
        // Not used in current flow; kept for future.
    }
    return true;
}

bool saveSelectedSignals(const char* path, const std::map<uint32_t, std::vector<DBCSignal>>& activeMap) {
    File file = SPIFFS.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("ERROR: Cannot open config file for writing");
        return false;
    }

    // Write opening bracket
    file.print('[');

    bool first = true;
    int count = 0;
    for (const auto& pair : activeMap) {
        for (const auto& sig : pair.second) {
            if (sig.isSelected) {
                if (!first) {
                    file.print(',');
                }
                first = false;

                // Write one signal object
                file.print("{\"id\":");
                file.print(pair.first);
                file.print(",\"signal\":\"");
                // Escape any double quotes in signal name (rare)
                String escaped = sig.name;
                escaped.replace("\"", "\\\"");
                file.print(escaped);
                file.print("\"}");

                count++;
                // Feed watchdog every 50 signals
                if (count % 50 == 0) {
                    esp_task_wdt_reset();
                    yield();
                }
            }
        }
    }

    // Write closing bracket
    file.print(']');
    file.close();

    Serial.printf("Saved %d selected signals to %s\n", count, path);
    return true;
}

void buildActiveMap(const std::vector<DBCMessage>& messages, std::map<uint32_t, std::vector<DBCSignal>>& activeMap) {
    activeMap.clear();
    for (const auto& msg : messages) {
        std::vector<DBCSignal> selected;
        for (const auto& sig : msg.signals) {
            if (sig.isSelected) {
                selected.push_back(sig);
            }
        }
        if (!selected.empty()) {
            activeMap[msg.id] = selected;
        }
    }
}

void clearActiveMap(std::map<uint32_t, std::vector<DBCSignal>>& activeMap) {
    activeMap.clear();
}
// Add to signal_selector.cpp (include required headers)
#include "dbc_parser.h"
#include <SPIFFS.h>
#include "config.h"

bool loadCANActiveMap(std::map<uint32_t, std::vector<DBCSignal>>& canActiveMap) {
    canActiveMap.clear();

    if (!SPIFFS.exists("/dbc/messages.json")) {
        Serial.println("No DBC messages found for CAN");
        return false;
    }
    if (!SPIFFS.exists(SIGNAL_CONFIG_PATH)) {
        Serial.println("No signal selection found for CAN");
        return false;
    }

    // Load all DBC messages
    std::vector<DBCMessage> messages;
    if (!loadDBCMessagesFromJson(messages, "/dbc/messages.json")) {
        Serial.println("Failed to load DBC messages");
        return false;
    }

    // Load selected signals list
    File file = SPIFFS.open(SIGNAL_CONFIG_PATH, FILE_READ);
    if (!file) {
        Serial.println("Failed to open signal config");
        return false;
    }
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
        Serial.println("Failed to parse signal config");
        return false;
    }

    // Build a map of message ID → set of selected signal names
    std::map<uint32_t, std::set<String>> selectedMap;
    JsonArray arr = doc.as<JsonArray>();
    for (JsonObject obj : arr) {
        uint32_t id = obj["id"];
        String sigName = obj["signal"].as<String>();
        selectedMap[id].insert(sigName);
    }

    // For each message, keep only the selected signals
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
            canActiveMap[msg.id] = selectedSignals;
        }
    }

    return !canActiveMap.empty();
}