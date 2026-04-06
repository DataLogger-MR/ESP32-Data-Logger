#include "dbc_parser.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>

// Forward declaration
bool parseSignalLine(const String& line, DBCSignal& sig, int lineNum);

bool parseDBCFile(fs::FS &fs, const char* path, std::vector<DBCMessage>& messages) {
    File file = fs.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open DBC file");
        return false;
    }

    messages.clear();
    DBCMessage currentMsg;
    bool inMessage = false;
    int lineNum = 0;
    String signalBuffer = "";          // for multi-line signals
    bool inSignal = false;              // true if we are collecting a multi-line signal

    while (file.available()) {
        String line = file.readStringUntil('\n');
        lineNum++;
        line.trim();                     // remove leading/trailing spaces

        if (line.length() == 0) continue;

        // Skip full-line comments (after trimming)
        if (line.startsWith("//")) continue;

        // Check for start of a new message
        if (line.startsWith("BO_ ")) {
            // If we were collecting a signal, finalize it (should not happen, but safe)
            if (inSignal && signalBuffer.length() > 0) {
                // This line is a new message, so the previous signal is incomplete – discard
                Serial.printf("Warning: Incomplete signal at line %d\n", lineNum);
                signalBuffer = "";
                inSignal = false;
            }

            // Save previous message if any
            if (inMessage) messages.push_back(currentMsg);
            inMessage = true;
            inSignal = false;

            unsigned long id;
            char name[64];
            int dlc;
            char transmitter[64];

            int matched = sscanf(line.c_str(), "BO_ %lu %[^:]: %d %s", &id, name, &dlc, transmitter);
            if (matched == 4) {
                currentMsg.id = (uint32_t)id & 0x1FFFFFFF;  // 29‑bit mask
                currentMsg.name = String(name);
                currentMsg.dlc = dlc;
                currentMsg.transmitter = String(transmitter);
                currentMsg.isExtended = (id > 0x7FF);       // rough heuristic
                currentMsg.signals.clear();
            } else {
                matched = sscanf(line.c_str(), "BO_ %lu %[^:]: %d", &id, name, &dlc);
                if (matched == 3) {
                    currentMsg.id = (uint32_t)id & 0x1FFFFFFF;
                    currentMsg.name = String(name);
                    currentMsg.dlc = dlc;
                    currentMsg.transmitter = "";
                    currentMsg.isExtended = (id > 0x7FF);
                    currentMsg.signals.clear();
                } else {
                    Serial.printf("Line %d: Malformed BO_: %s\n", lineNum, line.c_str());
                }
            }
        }
        // Handle signal lines
        else if (line.startsWith("SG_")) {
            // If we were already collecting a signal, the previous one was complete? Actually, a new SG_ means previous is done.
            if (inSignal && signalBuffer.length() > 0) {
                // Process the previously buffered signal
                DBCSignal sig;
                if (parseSignalLine(signalBuffer, sig, lineNum)) {
                    currentMsg.signals.push_back(sig);
                }
                signalBuffer = "";
                inSignal = false;
            }

            // Check if this line contains a colon – if yes, it's a complete signal
            if (line.indexOf(':') >= 0) {
                DBCSignal sig;
                if (parseSignalLine(line, sig, lineNum)) {
                    currentMsg.signals.push_back(sig);
                }
                inSignal = false;
            } else {
                // No colon, start multi-line accumulation
                signalBuffer = line;
                inSignal = true;
            }
        }
        // Continuation of a multi-line signal (line does not start with SG_ but we are inside a signal)
        else if (inSignal) {
            // Append this line to the buffer
            signalBuffer += " " + line;
            // If we now have a colon, the signal is complete
            if (signalBuffer.indexOf(':') >= 0) {
                DBCSignal sig;
                if (parseSignalLine(signalBuffer, sig, lineNum)) {
                    currentMsg.signals.push_back(sig);
                }
                signalBuffer = "";
                inSignal = false;
            }
        }
        // Otherwise, ignore (e.g., comments inside a message, attribute definitions, etc.)
    }

    // End of file: push last message and any pending signal
    if (inMessage) {
        if (inSignal && signalBuffer.length() > 0) {
            DBCSignal sig;
            if (parseSignalLine(signalBuffer, sig, lineNum)) {
                currentMsg.signals.push_back(sig);
            }
        }
        messages.push_back(currentMsg);
    }

    file.close();
    return true;
}

// Helper function to parse a complete signal line (or accumulated buffer)
bool parseSignalLine(const String& line, DBCSignal& sig, int lineNum) {
    // The line should contain "SG_ ... : ..."
    String rest = line;
    int sgPos = rest.indexOf("SG_");
    if (sgPos >= 0) {
        rest = rest.substring(sgPos + 3);
        rest.trim();
    }

    int colonPos = rest.indexOf(':');
    if (colonPos < 0) {
        Serial.printf("Line %d: No colon in SG_\n", lineNum);
        return false;
    }

    // Extract signal name (may include multiplexer indicator)
    String beforeColon = rest.substring(0, colonPos);
    beforeColon.trim();

    int spacePos = beforeColon.indexOf(' ');
    if (spacePos > 0) {
        String first = beforeColon.substring(0, spacePos);
        String second = beforeColon.substring(spacePos + 1);
        second.trim();
        if (second.length() == 1 && (second[0] == 'M' || second[0] == 'm'))
            sig.name = first;
        else
            sig.name = beforeColon;
    } else {
        sig.name = beforeColon;
    }

    String def = rest.substring(colonPos + 1);
    def.trim();

    // Parse bit definition: startBit|length@byteOrder+sign (factor,offset) [min|max] "unit" receiver
    int pipePos = def.indexOf('|');
    int atPos = def.indexOf('@');
    if (pipePos < 0 || atPos < 0) {
        Serial.printf("Line %d: Malformed bit definition\n", lineNum);
        return false;
    }

    sig.startBit = def.substring(0, pipePos).toInt();
    int lenEnd = atPos;
    String lenStr = def.substring(pipePos + 1, lenEnd);
    lenStr.trim();
    sig.length = lenStr.toInt();

    char byteOrder = def.charAt(atPos + 1);
    sig.isIntel = (byteOrder == '1');

    char signChar = def.charAt(atPos + 2);
    sig.isSigned = (signChar == '-');

    // Parse factor and offset
    int parenOpen = def.indexOf('(');
    int parenClose = def.indexOf(')', parenOpen);
    if (parenOpen < 0 || parenClose < 0) {
        Serial.printf("Line %d: Missing (factor,offset)\n", lineNum);
        return false;
    }

    String factorOffset = def.substring(parenOpen + 1, parenClose);
    factorOffset.replace(" ", "");
    int comma = factorOffset.indexOf(',');
    if (comma < 0) {
        Serial.printf("Line %d: Malformed factor/offset\n", lineNum);
        return false;
    }

    sig.scale = factorOffset.substring(0, comma).toFloat();
    sig.offset = factorOffset.substring(comma + 1).toFloat();

    // Parse min/max if present
    int bracketOpen = def.indexOf('[', parenClose);
    if (bracketOpen >= 0) {
        int bracketClose = def.indexOf(']', bracketOpen);
        if (bracketClose > bracketOpen) {
            String minMax = def.substring(bracketOpen + 1, bracketClose);
            int bar = minMax.indexOf('|');
            if (bar >= 0) {
                sig.minVal = minMax.substring(0, bar).toFloat();
                sig.maxVal = minMax.substring(bar + 1).toFloat();
            }
        }
    }

    // Parse unit if present
    int quote1 = def.indexOf('"', parenClose);
    if (quote1 >= 0) {
        int quote2 = def.indexOf('"', quote1 + 1);
        if (quote2 > quote1) {
            sig.unit = def.substring(quote1 + 1, quote2);
        }
    }

    // Parse receiver (last token)
    int lastSpace = def.lastIndexOf(' ');
    if (lastSpace > parenClose) {
        String possibleReceiver = def.substring(lastSpace + 1);
        possibleReceiver.trim();
        if (possibleReceiver.length() > 0 && possibleReceiver[0] != '[' && possibleReceiver[0] != '"') {
            sig.receiver = possibleReceiver;
        }
    }

    sig.isSelected = false;
    return true;
}

// ========== INCREMENTAL JSON SAVING ==========
bool saveDBCMessagesToJson(const std::vector<DBCMessage>& messages, const char* jsonPath) {
    File file = SPIFFS.open(jsonPath, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open JSON file for writing");
        return false;
    }

    // Write opening bracket
    file.print('[');

    for (size_t i = 0; i < messages.size(); i++) {
        if (i > 0) file.print(','); // comma between messages

        // Use a document sized for one message + its signals
        DynamicJsonDocument msgDoc(8192);
        JsonObject msgObj = msgDoc.to<JsonObject>();
        const auto& msg = messages[i];

        msgObj["id"] = msg.id;
        msgObj["hexId"] = "0x" + String(msg.id, HEX);
        msgObj["name"] = msg.name;
        msgObj["dlc"] = msg.dlc;
        msgObj["transmitter"] = msg.transmitter;
        msgObj["extended"] = msg.isExtended;

        JsonArray sigs = msgObj.createNestedArray("signals");
        for (const auto& sig : msg.signals) {
            JsonObject sigObj = sigs.createNestedObject();
            sigObj["name"] = sig.name;
            sigObj["startBit"] = sig.startBit;
            sigObj["length"] = sig.length;
            sigObj["intel"] = sig.isIntel;
            sigObj["signed"] = sig.isSigned;
            sigObj["scale"] = sig.scale;
            sigObj["offset"] = sig.offset;
            sigObj["min"] = sig.minVal;
            sigObj["max"] = sig.maxVal;
            sigObj["unit"] = sig.unit;
            sigObj["receiver"] = sig.receiver;
        }

        if (msgDoc.overflowed()) {
            Serial.printf("ERROR: Message document overflow for msg ID 0x%X\n", msg.id);
            file.close();
            SPIFFS.remove(jsonPath);
            return false;
        }

        // Serialize this message to the file
        serializeJson(msgDoc, file);
    }

    // Write closing bracket
    file.print(']');
    file.close();

    Serial.printf("JSON saved incrementally, %d messages\n", messages.size());
    return true;
}

// ========== STREAMING JSON LOADER ==========
bool loadDBCMessagesFromJsonStreaming(std::vector<DBCMessage>& messages, const char* jsonPath) {
    File file = SPIFFS.open(jsonPath, FILE_READ);
    if (!file) return false;

    messages.clear();
    char c;
    int braceLevel = 0;
    bool inString = false;
    char msgBuffer[16384];          // static buffer for one message JSON (16KB)
    int msgIdx = 0;
    bool inMessage = false;

    while (file.available()) {
        c = file.read();

        // Handle string boundaries (ignore braces inside strings)
        if (c == '"' && (msgIdx == 0 || msgBuffer[msgIdx-1] != '\\')) {
            inString = !inString;
        }

        if (!inString) {
            if (c == '{') {
                braceLevel++;
                if (braceLevel == 1) {
                    // Start of a new message object
                    msgIdx = 0;
                    msgBuffer[msgIdx++] = c;
                    inMessage = true;
                    continue;
                }
            } else if (c == '}') {
                braceLevel--;
                if (braceLevel == 0 && inMessage) {
                    // End of current message object
                    msgBuffer[msgIdx++] = c;
                    msgBuffer[msgIdx] = '\0'; // null-terminate

                    // Optional: log size of large messages
                    if (msgIdx > 8000) {
                        Serial.printf("Large message JSON: %d bytes\n", msgIdx);
                    }

                    // Parse this message with a document sized for one message
                    DynamicJsonDocument msgDoc(16384);
                    DeserializationError error = deserializeJson(msgDoc, msgBuffer);
                    if (error) {
                        Serial.printf("Failed to parse message JSON: %s, buffer size: %d\n", error.c_str(), msgIdx);
                        file.close();
                        return false;
                    }

                    JsonObject msgObj = msgDoc.as<JsonObject>();
                    DBCMessage msg;
                    msg.id = msgObj["id"];
                    msg.name = msgObj["name"].as<String>();
                    msg.dlc = msgObj["dlc"];
                    msg.isExtended = msgObj["extended"];
                    msg.transmitter = msgObj["transmitter"].as<String>();

                    JsonArray sigs = msgObj["signals"];
                    for (JsonObject sigObj : sigs) {
                        DBCSignal sig;
                        sig.name = sigObj["name"].as<String>();
                        sig.startBit = sigObj["startBit"];
                        sig.length = sigObj["length"];
                        sig.isIntel = sigObj["intel"];
                        sig.isSigned = sigObj["signed"] | false;
                        sig.scale = sigObj["scale"];
                        sig.offset = sigObj["offset"];
                        sig.minVal = sigObj["min"];
                        sig.maxVal = sigObj["max"];
                        sig.unit = sigObj["unit"].as<String>();
                        sig.receiver = sigObj["receiver"].as<String>();
                        sig.isSelected = false;
                        msg.signals.push_back(sig);
                    }
                    messages.push_back(msg);
                    inMessage = false;
                    continue;
                }
            }
        }

        if (inMessage) {
            if (msgIdx < sizeof(msgBuffer) - 1) {
                msgBuffer[msgIdx++] = c;
            } else {
                Serial.println("ERROR: Message JSON exceeds 16KB buffer");
                file.close();
                return false;
            }
        }
    }

    file.close();
    return true;
}

bool loadDBCMessagesFromJson(std::vector<DBCMessage>& messages, const char* jsonPath) {
    // First try with a large document (192KB)
    File file = SPIFFS.open(jsonPath, FILE_READ);
    if (!file) return false;

    DynamicJsonDocument doc(196608); // 192KB
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        if (error == DeserializationError::NoMemory) {
            Serial.println("Normal JSON load failed (NoMemory), falling back to streaming loader");
            return loadDBCMessagesFromJsonStreaming(messages, jsonPath);
        }
        Serial.printf("Failed to load JSON: %s\n", error.c_str());
        return false;
    }

    messages.clear();
    JsonArray arr = doc.as<JsonArray>();
    for (JsonObject msgObj : arr) {
        DBCMessage msg;
        msg.id = msgObj["id"];
        msg.name = msgObj["name"].as<String>();
        msg.dlc = msgObj["dlc"];
        msg.isExtended = msgObj["extended"];
        msg.transmitter = msgObj["transmitter"].as<String>();
        JsonArray sigs = msgObj["signals"];
        for (JsonObject sigObj : sigs) {
            DBCSignal sig;
            sig.name = sigObj["name"].as<String>();
            sig.startBit = sigObj["startBit"];
            sig.length = sigObj["length"];
            sig.isIntel = sigObj["intel"];
            sig.isSigned = sigObj["signed"] | false;
            sig.scale = sigObj["scale"];
            sig.offset = sigObj["offset"];
            sig.minVal = sigObj["min"];
            sig.maxVal = sigObj["max"];
            sig.unit = sigObj["unit"].as<String>();
            sig.receiver = sigObj["receiver"].as<String>();
            sig.isSelected = false;
            msg.signals.push_back(sig);
        }
        messages.push_back(msg);
    }
    return true;
}

void freeDBCMessages(std::vector<DBCMessage>& messages) {
    messages.clear();
}