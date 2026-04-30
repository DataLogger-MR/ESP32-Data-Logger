#ifndef CT_CAN_SENSORS_H
#define CT_CAN_SENSORS_H

#include <Arduino.h>
#include "driver/twai.h"

// ================ CAN IDs for CT-CAN Sensors ================
#define CAN_ID_FLOW             0x181    
#define CAN_ID_PRESSURE         0x182    
#define CAN_ID_TEMP             0x183    
#define CAN_ID_NMT              0x000    

// ================ NMT Commands ================
#define NMT_START_NODE          0x01
#define NMT_STOP_NODE           0x02
#define NMT_PRE_OPERATIONAL     0x80

// ================ Status Values ================
#define CT_STATUS_VALID         0x00
#define CT_STATUS_BROKEN_WIRE   0x01
#define CT_STATUS_RANGE_HIGH    0x02
#define CT_STATUS_RANGE_LOW     0x04

// ================ Data Structures ================
struct CTFlowData {
    float flow_lpm;
    float flow_gpm;
    uint8_t status;
    bool valid;
    unsigned long lastUpdate;
    unsigned long timeoutMs;
};

struct CTPressureData {
    float pressure_bar;
    float pressure_psi;
    uint8_t status;
    bool valid;
    unsigned long lastUpdate;
    unsigned long timeoutMs;
};

struct CTTemperatureData {
    float temp_celsius;
    float temp_fahrenheit;
    uint8_t status;
    bool valid;
    unsigned long lastUpdate;
    unsigned long timeoutMs;
};

// ================ Global Instances ================
extern CTFlowData ctFlow;
extern CTPressureData ctPressure;
extern CTTemperatureData ctTemp;
extern bool ctSensorsInitialized;

// ================ Function Prototypes ================
void initCTSensors();
void sendNMTStartCommand();
void processCTCANMessage(const twai_message_t& msg);
String getCTStatusString(uint8_t status);
void updateCTSensors();
bool isCTDataValid(unsigned long lastUpdate, unsigned long timeoutMs, unsigned long currentTime);

#endif