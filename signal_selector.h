#ifndef SIGNAL_SELECTOR_H
#define SIGNAL_SELECTOR_H

#include <Arduino.h>
#include <vector>
#include <map>
#include "types.h"

bool loadSelectedSignals(const char* path, std::map<uint32_t, std::vector<DBCSignal>>& activeMap);
bool saveSelectedSignals(const char* path, const std::map<uint32_t, std::vector<DBCSignal>>& activeMap);
void buildActiveMap(const std::vector<DBCMessage>& messages, std::map<uint32_t, std::vector<DBCSignal>>& activeMap);
void clearActiveMap(std::map<uint32_t, std::vector<DBCSignal>>& activeMap);
// Add to signal_selector.h
bool loadCANActiveMap(std::map<uint32_t, std::vector<DBCSignal>>& canActiveMap);

#endif