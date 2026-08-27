#ifndef ADS1115_SCHEDULER_H
#define ADS1115_SCHEDULER_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>   // for SemaphoreHandle_t

// ADS1115 I2C addresses
#define ADS1115_ADDR_48  0x48
#define ADS1115_ADDR_4B  0x4B
#define ADS1115_ADDR_49  0x49
#define ADS1115_ADDR_4A  0x4A

// Number of devices and channels
#define ADS1115_DEVICES  4
#define ADS1115_CHANNELS 4

// Structure to hold latest ADC readings (raw + voltage)
struct ADS1115Reading {
    int16_t raw[ADS1115_CHANNELS];
    float voltage[ADS1115_CHANNELS];
    bool valid[ADS1115_CHANNELS];
    unsigned long lastUpdate;
};

// Global readings for each address (index 0..3 maps to 0x48, 0x4B, 0x49, 0x4A)
extern ADS1115Reading ads1115Readings[ADS1115_DEVICES];

// Global I2C mutex – shared by all I2C users
extern SemaphoreHandle_t i2cMutex;

// Initialize the scheduler task; call once in setup()
void initADS1115Scheduler();

// Get a pointer to the readings for a given address (returns NULL if not found)
const ADS1115Reading* getADS1115Reading(uint8_t address);

#endif