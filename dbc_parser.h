#ifndef DBC_PARSER_H
#define DBC_PARSER_H

#include <Arduino.h>
#include <vector>
#include "types.h"
#include "FS.h"

bool parseDBCFile(fs::FS &fs, const char* path, std::vector<DBCMessage>& messages);
bool saveDBCMessagesToJson(const std::vector<DBCMessage>& messages, const char* jsonPath);
bool loadDBCMessagesFromJson(std::vector<DBCMessage>& messages, const char* jsonPath);
void freeDBCMessages(std::vector<DBCMessage>& messages);

#endif