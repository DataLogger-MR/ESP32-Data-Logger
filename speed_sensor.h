#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <Arduino.h>
#include "pins.h"

// Speed sensor configuration
#define SPEED_SENSOR_PIN    Encoder_PWM  // GPIO12 from pins.h
#define PULSES_PER_REVOLUTION 5         // Adjust based on your magnet setup
#define SPEED_UPDATE_INTERVAL_MS 100    // Update every 100ms

// Industrial filter settings
#define LPF_CUTOFF_FREQ_HZ 5.0f         // 5Hz cutoff (removes noise above 5Hz)
#define MOVING_AVERAGE_WINDOW 5          // 5-sample moving average
#define MEDIAN_FILTER_WINDOW 3           // 3-point median filter for spike removal

// Speed data structure
struct SpeedData {
    float rpm;                  
    float frequency_hz;         
    float speed_kmh;            
    float speed_mps;            
    float raw_frequency;        // Raw frequency before filtering
    unsigned long lastUpdate;   
    bool valid;                 
    unsigned long timeoutMs;    
};

// Debug structure
typedef struct {
    unsigned long lastUpdateTime;
    unsigned long lastPulseTime;
    int pulseCount;
    float rawFreq[10];
    float filteredFreq[10];
    float lpfOutput[10];
    float movingAvgOutput[10];
    int historyIndex;
    bool isrTriggered;
    unsigned long lastIsrTime;
} SpeedDebugData;

// Function prototypes
void initSpeedSensor();
void IRAM_ATTR speedSensorISR();
void updateSpeed();
float getCurrentRPM();
float getCurrentFrequency();
float getCurrentSpeedKMH();
void setPulsesPerRevolution(uint8_t pulses);
void setWheelDiameter(float diameter_mm);
void setLowPassFilterCutoff(float cutoffFreq);
void setMovingAverageWindow(uint8_t windowSize);

// Debug functions
void debugSpeedSensor();
void resetDebugCounters();

// Sample-and-hold functions
float getLastValidSpeed();
unsigned long getTimeSinceLastPulse();
bool hasRecentValidData(unsigned long maxAgeMs);

// Global declarations
extern SpeedData speedData;
extern volatile unsigned long speedPulseCount;
extern volatile bool speedPulseDetected;
extern SpeedDebugData speedDebug;

#endif