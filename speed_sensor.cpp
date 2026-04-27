#include "speed_sensor.h"
#include "globals.h"

volatile unsigned long lastPulseMicros = 0;
volatile unsigned long currentPeriodMicros = 0;
volatile bool newPulseReady = false;

static unsigned long lastPulseTime = 0;

static float lpfPreviousOutput = 0;
static bool lpfInitialized = false;

static float lowPassFilter(float input) {
    if (!lpfInitialized) {
        lpfPreviousOutput = input;
        lpfInitialized = true;
        return input;
    }
    
    lpfPreviousOutput = (LPF_ALPHA * input) + ((1.0f - LPF_ALPHA) * lpfPreviousOutput);
    return lpfPreviousOutput;
}

void IRAM_ATTR speedSensorISR() {
    unsigned long now = micros();
    unsigned long period = now - lastPulseMicros;
    
    if (period > 500 && period < 100000) {
        currentPeriodMicros = period;
        newPulseReady = true;
    }
    lastPulseMicros = now;
}

void initSpeedSensor() {
    pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), speedSensorISR, RISING);
    
    speedData.rpm = 0;
    speedData.lastUpdate = 0;
    speedData.valid = false;
    speedData.timeoutMs = 1000;
    
    lastPulseMicros = 0;
    currentPeriodMicros = 0;
    newPulseReady = false;
    
    lpfPreviousOutput = 0;
    lpfInitialized = false;
}

void updateSpeed() {
    unsigned long now = millis();
    
    if (newPulseReady) {
        newPulseReady = false;
        
        float frequency = 1000000.0f / (float)currentPeriodMicros;
        
        float rawRpm = (frequency * 60.0f) / (float)PULSES_PER_REVOLUTION;
        
        if (rawRpm < 0) rawRpm = 0;
        if (rawRpm > 15000) rawRpm = 15000;
        
        speedData.rpm = lowPassFilter(rawRpm);
        
        speedData.lastUpdate = now;
        speedData.valid = true;
        lastPulseTime = now;
    }
    
    if (now - lastPulseTime > 1000) {
        if (speedData.valid) {
            speedData.valid = false;
            speedData.rpm = 0;
            lpfInitialized = false;  
        }
    }
}

float getCurrentRPM() {
    return speedData.rpm;
}