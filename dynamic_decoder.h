#ifndef DYNAMIC_DECODER_H
#define DYNAMIC_DECODER_H

#include <Arduino.h>
#include <map>
#include <vector>
#include "types.h"
#include "driver/twai.h"

extern std::map<uint32_t, std::vector<DBCSignal>> activeSignals;
extern std::map<String, unsigned long> lastDynamicUpdateTime;

void initDynamicDecoder();
void setActiveSignals(const std::map<uint32_t, std::vector<DBCSignal>>& newMap);
double extractSignalValue(const uint8_t* data, uint8_t startBit, uint8_t length, bool intel, bool isSigned);
void decodeDynamic(const twai_message_t& msg, std::map<String, double>& outValues);
String buildDynamicCSVHeader();
String buildDynamicCSVRow(unsigned long timestamp, const std::map<String, double>& values);

// NEW
void initDynamicValues(const std::map<uint32_t, std::vector<DBCSignal>>& activeMap);

#endif