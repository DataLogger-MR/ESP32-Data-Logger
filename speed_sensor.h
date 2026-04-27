#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <Arduino.h>
#include "pins.h"

#define SPEED_SENSOR_PIN Encoder_PWM
#define PULSES_PER_REVOLUTION 5

#define LPF_ALPHA 0.15f  

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

#endif