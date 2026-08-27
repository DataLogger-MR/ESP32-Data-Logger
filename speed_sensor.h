#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <Arduino.h>
#include "pins.h"

#define SPEED_SENSOR_PIN Encoder_PWM
#define AUX_SPEED_SENSOR_PIN AUX_Encoder_PWM

#define PULSES_PER_REVOLUTION 5
#define AUX_PULSES_PER_REVOLUTION 5   // Change if different

#define LPF_ALPHA 0.15f

// ================ MAIN SPEED ================
struct SpeedData {
    float rpm;
    unsigned long lastUpdate;
    bool valid;
    unsigned long timeoutMs;
};

extern SpeedData speedData;
extern volatile unsigned long lastPulseMicros;
extern volatile bool newPulseReady;

void initSpeedSensor();
void IRAM_ATTR speedSensorISR();
void updateSpeed();
float getCurrentRPM();

// ================ AUX SPEED ================  // NEW
struct AuxSpeedData {
    float rpm;
    unsigned long lastUpdate;
    bool valid;
    unsigned long timeoutMs;
};

extern AuxSpeedData auxSpeedData;
extern volatile unsigned long lastAuxPulseMicros;
extern volatile bool newAuxPulseReady;

void initAuxSpeedSensor();
void IRAM_ATTR auxSpeedSensorISR();
void updateAuxSpeed();
float getAuxRPM();

#endif