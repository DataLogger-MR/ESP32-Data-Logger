#include "speed_sensor.h"
#include "globals.h"

// ================ MAIN SPEED SENSOR ================
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

// ================ AUX SPEED SENSOR (NEW) ================
volatile unsigned long lastAuxPulseMicros = 0;
volatile unsigned long currentAuxPeriodMicros = 0;
volatile bool newAuxPulseReady = false;

static unsigned long lastAuxPulseTime = 0;
static float auxLpfPreviousOutput = 0;
static bool auxLpfInitialized = false;

static float auxLowPassFilter(float input) {
    if (!auxLpfInitialized) {
        auxLpfPreviousOutput = input;
        auxLpfInitialized = true;
        return input;
    }
    auxLpfPreviousOutput = (LPF_ALPHA * input) + ((1.0f - LPF_ALPHA) * auxLpfPreviousOutput);
    return auxLpfPreviousOutput;
}

void IRAM_ATTR auxSpeedSensorISR() {
    unsigned long now = micros();
    unsigned long period = now - lastAuxPulseMicros;
    if (period > 500 && period < 100000) {
        currentAuxPeriodMicros = period;
        newAuxPulseReady = true;
    }
    lastAuxPulseMicros = now;
}

void initAuxSpeedSensor() {
    pinMode(AUX_SPEED_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(AUX_SPEED_SENSOR_PIN), auxSpeedSensorISR, RISING);
    auxSpeedData.rpm = 0;
    auxSpeedData.lastUpdate = 0;
    auxSpeedData.valid = false;
    auxSpeedData.timeoutMs = 1000;
    lastAuxPulseMicros = 0;
    currentAuxPeriodMicros = 0;
    newAuxPulseReady = false;
    auxLpfPreviousOutput = 0;
    auxLpfInitialized = false;
}

void updateAuxSpeed() {
    unsigned long now = millis();
    if (newAuxPulseReady) {
        newAuxPulseReady = false;
        float frequency = 1000000.0f / (float)currentAuxPeriodMicros;
        float rawRpm = (frequency * 60.0f) / (float)AUX_PULSES_PER_REVOLUTION;
        if (rawRpm < 0) rawRpm = 0;
        if (rawRpm > 15000) rawRpm = 15000;
        auxSpeedData.rpm = auxLowPassFilter(rawRpm);
        auxSpeedData.lastUpdate = now;
        auxSpeedData.valid = true;
        lastAuxPulseTime = now;
    }
    if (now - lastAuxPulseTime > 1000) {
        if (auxSpeedData.valid) {
            auxSpeedData.valid = false;
            auxSpeedData.rpm = 0;
            auxLpfInitialized = false;
        }
    }
}

float getAuxRPM() {
    return auxSpeedData.rpm;
}