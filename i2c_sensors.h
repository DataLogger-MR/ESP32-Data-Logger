#ifndef I2C_SENSORS_H
#define I2C_SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP9600.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <RTClib.h>
#include "i2c_config.h"
#include <map>

// I2C Multiplexer (PCA9548A) Addresses
#define PCA9548A_ADDR_MAIN    0x70
#define PCA9548A_ADDR_SECONDARY 0x74

// Maximum number of thermocouples supported
#define THERMOCOUPLE_COUNT 16

// ADC channels on ADS1115
#define ADC_CHANNEL_COUNT 4

// ========== I2C Sensor Data Structures ==========

struct DS3231Data {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    float temperature;
    bool valid;
    unsigned long lastUpdate;
};

struct ThermocoupleData {
    float temperature;      // °C
    float ambientTemp;      // °C (MCP9600 internal)
    uint8_t fault;          // Fault status (0 = no fault)
    bool valid;
    unsigned long lastUpdate;
};

struct MCP23017Data {
    uint16_t gpio_state;    // 16-bit GPIO state
    uint8_t input_pins[16]; // Individual pin states
    bool valid;
    unsigned long lastUpdate;
};

// ADC data structure for one ADS1115 device
struct ADS1115Data {
    float voltages[4];
    int16_t raw_values[4];
    bool valid;
    unsigned long lastUpdate;
};

// Main sensor data structure
struct I2CSensorData {
    DS3231Data rtc;
    ThermocoupleData thermocouples[THERMOCOUPLE_COUNT];
    MCP23017Data gpio;
    std::map<uint8_t, ADS1115Data> ads1115_data;   // key = I2C address
    bool mcp9600_present[THERMOCOUPLE_COUNT];
    bool mcp23017_present;
    bool ads1115_present;   // true if at least one ADS1115 is present
    bool pca9548a_present;
    unsigned long lastScanTime;
};

extern I2CSensorData sensorData;
extern bool useMultiplexer;
extern bool hasSecondaryMux;

// Function prototypes
void initI2CSensors();
void scanI2CBus();
bool selectMainMuxChannel(uint8_t channel);
bool selectSecondaryMuxChannel(uint8_t channel);
bool selectI2CPath(uint8_t mainChannel, int8_t secondaryChannel = -1);
void resetI2CPath();
void updateI2CSensors();
void formatI2CData(char* buffer, size_t bufferSize, unsigned long currentTime);

// Individual sensor update functions
void updateRTC();
void updateThermocouples();
void updateMCP23017();
void updateADS1115();   // now reads all configured ADS1115 devices

// Helper macro for thermocouple names (optional)
#define THERMOCOUPLE_NAMES { \
    "TC1_Battery_Cell1", "TC2_Battery_Cell2", "TC3_Battery_Cell3", "TC4_Battery_Cell4", \
    "TC5_Battery_Cell5", "TC6_Battery_Cell6", "TC7_Motor_Winding", "TC8_Motor_Bearing", \
    "TC9_Controller", "TC10_Ambient", "TC11_Extra", "TC12_Extra2" \
}

#endif // I2C_SENSORS_H