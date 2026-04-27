#include "speed_sensor.h"
#include "globals.h"

// Global variables
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long currentPeriodMicros = 0;
volatile bool newPulseReady = false;

static unsigned long lastPulseTime = 0;

// Low Pass Filter variables
static float lpfPreviousOutput = 0;
static bool lpfInitialized = false;

// Simple 1st order Low Pass Filter
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
    
    // Valid pulse range: 500us to 100ms (10Hz to 2000Hz)
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
    
    // Reset filter
    lpfPreviousOutput = 0;
    lpfInitialized = false;
}

void updateSpeed() {
    unsigned long now = millis();
    
    if (newPulseReady) {
        newPulseReady = false;
        
        // Calculate raw frequency from period
        float frequency = 1000000.0f / (float)currentPeriodMicros;
        
        // Calculate raw RPM
        float rawRpm = (frequency * 60.0f) / (float)PULSES_PER_REVOLUTION;
        
        // Limit raw RPM to reasonable range (0 - 15000 RPM)
        if (rawRpm < 0) rawRpm = 0;
        if (rawRpm > 15000) rawRpm = 15000;
        
        // Apply Low Pass Filter to RPM and store back to speedData.rpm
        speedData.rpm = lowPassFilter(rawRpm);
        
        speedData.lastUpdate = now;
        speedData.valid = true;
        lastPulseTime = now;
    }
    
    // Timeout after 1 second of no pulses
    if (now - lastPulseTime > 1000) {
        if (speedData.valid) {
            speedData.valid = false;
            speedData.rpm = 0;
            lpfInitialized = false;  // Reset filter for next start
        }
    }
}

float getCurrentRPM() {
    return speedData.rpm;
}