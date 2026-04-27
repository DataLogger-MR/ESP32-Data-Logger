#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include <Arduino.h>
#include <map>
#include <vector>

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
    int8_t channel = -1;   
};

struct I2CDeviceConfig {
    String type;                      
    uint8_t address;                  
    std::vector<I2CSignalConfig> signals;
};

struct I2CConfig {
    std::vector<I2CDeviceConfig> devices;
};

extern I2CConfig i2cConfig;
extern bool i2cConfigLoaded;

void loadI2CConfig();
bool saveI2CConfig(const I2CConfig& config);

bool parseI2CConfigCSV(const String& csv, I2CConfig& config);
String i2cConfigToCSV(const I2CConfig& config);

#endif