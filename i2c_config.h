#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include <Arduino.h>
#include <map>
#include <vector>

// Configuration for a single signal (from any I2C device)
struct I2CSignalConfig {
    String name;
    bool enabled;
    float factor;
    float offset;
    String unit;
    uint16_t bitMask;
    bool isMapped;
    std::map<uint16_t, String> valueMapping;
    bool invert;
    int8_t channel = -1;   // For ADS1115: 0..3, for MCP23017: bit position, for others unused
};

// Configuration for a single I2C device
struct I2CDeviceConfig {
    String type;                      // "MCP9600", "ADS1115", "MCP23017", "DS3231" etc.
    uint8_t address;                  // I2C address in decimal
    std::vector<I2CSignalConfig> signals;
};

// Complete I2C configuration
struct I2CConfig {
    std::vector<I2CDeviceConfig> devices;
};

extern I2CConfig i2cConfig;
extern bool i2cConfigLoaded;

// Load/save from/to SPIFFS (JSON)
void loadI2CConfig();
bool saveI2CConfig(const I2CConfig& config);

// CSV import/export
bool parseI2CConfigCSV(const String& csv, I2CConfig& config);
String i2cConfigToCSV(const I2CConfig& config);

#endif