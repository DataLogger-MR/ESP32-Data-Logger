#include "ads1115_scheduler.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>

// ---- Additional includes for slot tasks ----
#include "gps_manager.h"
#include "i2c_sensors.h"
#include "wifi_manager.h"
#include "speed_sensor.h"
#include "globals.h"
#include "torque_rs485.h"   // <-- TORQUE: include torque header

// ------------------------------------------------------------
// Global mutex for I2C (shared with other I2C users)
// ------------------------------------------------------------
SemaphoreHandle_t i2cMutex = NULL;

// ------------------------------------------------------------
// Global readings storage
// ------------------------------------------------------------
ADS1115Reading ads1115Readings[ADS1115_DEVICES];

// ---- NEW: Averaging accumulators ----
struct ADCAccumulator {
    float sum[ADS1115_CHANNELS];    // Sum of readings for each channel
    int count[ADS1115_CHANNELS];    // Number of readings accumulated
    bool hasData[ADS1115_CHANNELS]; // Whether we have any data for this channel
};

static ADCAccumulator accumulators[ADS1115_DEVICES];

// Map address to index
static uint8_t addrToIndex[ADS1115_DEVICES] = { 
    ADS1115_ADDR_48, ADS1115_ADDR_4B, ADS1115_ADDR_49, ADS1115_ADDR_4A 
};

// Adafruit objects (used only for gain setting, not for reading)
static Adafruit_ADS1115 ads[ADS1115_DEVICES];

// Schedule tables (trigger channels per tick)
static const int8_t triggerSchedule[8][ADS1115_DEVICES] = {
    {0, 0, 0, 0},   // t=0: all A0
    {1, 1, 0, 0},   // t=10: 0x48:A1, 0x4B:A1, 0x49:A0, 0x4A:A0
    {2, 2, 0, 0},
    {3, 2, 0, 0},
    {0, 0, 0, 0},
    {1, 1, 0, 0},
    {2, 2, 0, 0},
    {3, 2, 0, 0}
};

// Last triggered channel per device (for reading next tick)
static int8_t lastTrigger[ADS1115_DEVICES] = {-1, -1, -1, -1};

// ---- NEW: Cycle tracking ----
static uint32_t currentCycleNumber = 0;

// ------------------------------------------------------------
// Low-level I2C helpers (with mutex protection)
// ------------------------------------------------------------
static bool triggerConversion(uint8_t devIdx, uint8_t channel) {
    uint8_t addr = addrToIndex[devIdx];
    uint16_t config = 0x8000;
    config |= ((0x4 + channel) << 12);
    config |= 0x0200;
    config |= 0x0100;
    config |= 0x0080;
    config |= 0x0003;

    Wire.beginTransmission(addr);
    Wire.write(0x01);
    Wire.write((config >> 8) & 0xFF);
    Wire.write(config & 0xFF);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    return true;
}

static bool readConversion(uint8_t devIdx, uint8_t channel, int16_t &raw, float &voltage) {
    uint8_t addr = addrToIndex[devIdx];
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(addr, (uint8_t)2) != 2) {
        return false;
    }
    uint16_t val = (Wire.read() << 8) | Wire.read();
    raw = (int16_t)val;
    voltage = raw * 0.000125f;
    return true;
}

// ---- NEW: Reset accumulators for a device ----
static void resetAccumulator(int devIdx) {
    for (int ch = 0; ch < ADS1115_CHANNELS; ch++) {
        accumulators[devIdx].sum[ch] = 0.0f;
        accumulators[devIdx].count[ch] = 0;
        accumulators[devIdx].hasData[ch] = false;
    }
}

// ---- NEW: Reset all accumulators ----
static void resetAllAccumulators() {
    for (int dev = 0; dev < ADS1115_DEVICES; dev++) {
        resetAccumulator(dev);
    }
}

// ---- NEW: Add a reading to the accumulator ----
static void addToAccumulator(int devIdx, int channel, float voltage) {
    if (channel < 0 || channel >= ADS1115_CHANNELS) return;
    accumulators[devIdx].sum[channel] += voltage;
    accumulators[devIdx].count[channel]++;
    accumulators[devIdx].hasData[channel] = true;
}

// ---- NEW: Compute averages and store in ads1115Readings ----
static void computeAndStoreAverages() {
    for (int dev = 0; dev < ADS1115_DEVICES; dev++) {
        for (int ch = 0; ch < ADS1115_CHANNELS; ch++) {
            if (accumulators[dev].hasData[ch] && accumulators[dev].count[ch] > 0) {
                float avg = accumulators[dev].sum[ch] / accumulators[dev].count[ch];
                ads1115Readings[dev].voltage[ch] = avg;
                ads1115Readings[dev].raw[ch] = (int16_t)(avg / 0.000125f);
                ads1115Readings[dev].valid[ch] = true;
                ads1115Readings[dev].lastUpdate = millis();
            }
        }
    }
}

// ------------------------------------------------------------
// Slot task implementation – Phase 3 (UPDATED: slot 8 removed torque send)
// ------------------------------------------------------------
static void performSlotTask(int slotIdx) {
    unsigned long start = micros();

    switch (slotIdx) {
        case 0:
            updateGPS();
            updateCompass();
            updateRTC();
            break;

        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
            readThermocouplePair(slotIdx - 1);
            break;

        case 7:
            updateMCP23017();
            break;

        case 8:
            break;

        case 9:
            break;

        default:
            break;
    }

    unsigned long duration = micros() - start;
    // duration check removed (no print)
}

// ------------------------------------------------------------
// Scheduler Task – 100 ms cycle with 20 ms slot
// ------------------------------------------------------------
static void ads1115SchedulerTask(void *pvParameters) {
    TickType_t nextWake = xTaskGetTickCount() + pdMS_TO_TICKS(10);
    const TickType_t tickPeriod = pdMS_TO_TICKS(10);
    const TickType_t slotPeriod = pdMS_TO_TICKS(20);

    uint32_t cycleCounter = 0;
    int slotIdx = 0;

    resetAllAccumulators();

    while (1) {
        for (int tick = 0; tick < 8; tick++) {
            vTaskDelayUntil(&nextWake, tickPeriod);

            if (tick > 0) {
                for (int dev = 0; dev < ADS1115_DEVICES; dev++) {
                    int8_t ch = lastTrigger[dev];
                    if (ch >= 0 && ch < ADS1115_CHANNELS) {
                        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            int16_t raw;
                            float voltage;
                            bool ok = readConversion(dev, ch, raw, voltage);
                            if (ok) {
                                addToAccumulator(dev, ch, voltage);
                            } else {
                                accumulators[dev].hasData[ch] = false;
                            }
                            xSemaphoreGive(i2cMutex);
                        } else {
                            // mutex timeout – no print
                        }
                    }
                }
            }

            for (int dev = 0; dev < ADS1115_DEVICES; dev++) {
                int8_t ch = triggerSchedule[tick][dev];
                if (ch >= 0 && ch < ADS1115_CHANNELS) {
                    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        bool ok = triggerConversion(dev, ch);
                        lastTrigger[dev] = ok ? ch : -1;
                        xSemaphoreGive(i2cMutex);
                    } else {
                        lastTrigger[dev] = -1;
                    }
                } else {
                    lastTrigger[dev] = -1;
                }
            }

            // Torque service - reads any available data from Serial2 (aux sensor)
            torque_service();
        }

        computeAndStoreAverages();

        performSlotTask(slotIdx);

        populateI2CValues();

        // REMOVED: torque_send() - aux sensor sends data automatically every 100ms
        // The sensor is now read-only on Serial2, no trigger needed.

        vTaskDelayUntil(&nextWake, slotPeriod);

        slotIdx = (slotIdx + 1) % 10;
        cycleCounter++;
        currentCycleNumber++;

        resetAllAccumulators();

        // debug prints removed
    }
}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
void initADS1115Scheduler() {
    if (i2cMutex == NULL) {
        i2cMutex = xSemaphoreCreateMutex();
        if (i2cMutex == NULL) {
            // mutex creation failed – no print
            return;
        }
    }

    for (int i = 0; i < ADS1115_DEVICES; i++) {
        if (!ads[i].begin(addrToIndex[i])) {
            // device not found – no print
        } else {
            ads[i].setGain(GAIN_ONE);
            ads[i].setDataRate(RATE_ADS1115_128SPS);
        }
    }

    for (int i = 0; i < ADS1115_DEVICES; i++) {
        for (int ch = 0; ch < ADS1115_CHANNELS; ch++) {
            ads1115Readings[i].valid[ch] = false;
            ads1115Readings[i].voltage[ch] = 0.0f;
            ads1115Readings[i].raw[ch] = 0;
        }
        ads1115Readings[i].lastUpdate = 0;
        resetAccumulator(i);
    }

    currentCycleNumber = 0;

    xTaskCreatePinnedToCore(
        ads1115SchedulerTask,
        "ADS1115Sched",
        4096,
        NULL,
        2,
        NULL,
        0
    );
    // startup message removed
}

const ADS1115Reading* getADS1115Reading(uint8_t address) {
    for (int i = 0; i < ADS1115_DEVICES; i++) {
        if (addrToIndex[i] == address) {
            return &ads1115Readings[i];
        }
    }
    return NULL;
}