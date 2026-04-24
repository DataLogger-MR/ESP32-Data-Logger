#include "speed_sensor.h"
#include "globals.h"
#include "session_manager.h"
#include "file_manager.h"
#include <math.h>
#include <string.h>

// ========== GLOBAL VARIABLES ==========
volatile unsigned long speedPulseCount = 0;
volatile bool speedPulseDetected = false;
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long currentPeriodMicros = 0;
volatile bool newPulseReady = false;

// Configuration
static float wheelDiameter_mm = 500.0;
static const unsigned long MIN_PULSE_INTERVAL_US = 500;    // 2kHz max
static const unsigned long MAX_PULSE_INTERVAL_US = 100000; // 10Hz min

// ========== INDUSTRIAL FILTERS ==========

// 1. First-Order Low-Pass Filter (RC equivalent)
typedef struct {
    float output;
    float alpha;  // Filter coefficient (0-1)
    bool initialized;
} LowPassFilter;

static LowPassFilter lpf = {0, 0, false};

// Initialize LPF with cutoff frequency (fc) and sample rate (fs)
void initLowPassFilter(float cutoffFreq_Hz, float sampleRate_Hz) {
    float RC = 1.0f / (2.0f * 3.14159265f * cutoffFreq_Hz);
    float dt = 1.0f / sampleRate_Hz;
    lpf.alpha = dt / (RC + dt);
    lpf.initialized = false;
    Serial.printf("[FILTER] LPF initialized: cutoff=%.1fHz, alpha=%.3f\n", cutoffFreq_Hz, lpf.alpha);
}

float applyLowPassFilter(float input) {
    if (!lpf.initialized) {
        lpf.output = input;
        lpf.initialized = true;
    } else {
        lpf.output = lpf.alpha * input + (1.0f - lpf.alpha) * lpf.output;
    }
    return lpf.output;
}

// 2. Moving Average Filter
typedef struct {
    float buffer[20];  // Max 20 samples
    uint8_t size;
    uint8_t index;
    float sum;
    uint8_t count;
} MovingAverage;

static MovingAverage movAvg = {0};

void initMovingAverage(uint8_t windowSize) {
    movAvg.size = (windowSize > 20) ? 20 : windowSize;
    movAvg.index = 0;
    movAvg.sum = 0;
    movAvg.count = 0;
    memset(movAvg.buffer, 0, sizeof(movAvg.buffer));
    Serial.printf("[FILTER] Moving Average initialized: window=%d\n", movAvg.size);
}

float applyMovingAverage(float input) {
    if (movAvg.size == 0) return input;
    
    // Remove oldest value if buffer is full
    if (movAvg.count >= movAvg.size) {
        movAvg.sum -= movAvg.buffer[movAvg.index];
    } else {
        movAvg.count++;
    }
    
    // Add new value
    movAvg.buffer[movAvg.index] = input;
    movAvg.sum += input;
    movAvg.index = (movAvg.index + 1) % movAvg.size;
    
    return movAvg.sum / movAvg.count;
}

// 3. Median Filter (for spike removal)
float applyMedianFilter(float* buffer, uint8_t size, float newValue) {
    static float medianBuffer[10];
    static uint8_t medianIndex = 0;
    static uint8_t medianCount = 0;
    
    // Add new value to circular buffer
    medianBuffer[medianIndex] = newValue;
    medianIndex = (medianIndex + 1) % size;
    if (medianCount < size) medianCount++;
    
    // Copy to temporary array for sorting
    float temp[10];
    for (int i = 0; i < medianCount; i++) {
        temp[i] = medianBuffer[i];
    }
    
    // Simple bubble sort
    for (int i = 0; i < medianCount - 1; i++) {
        for (int j = 0; j < medianCount - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }
    
    return temp[medianCount / 2];
}

// 4. Rate Limiter (prevents sudden jumps)
typedef struct {
    float lastOutput;
    float maxChangePerSecond;
    unsigned long lastTime;
    bool initialized;
} RateLimiter;

static RateLimiter rateLim = {0, 0, 0, false};

void initRateLimiter(float maxChange_Hz_per_sec) {
    rateLim.maxChangePerSecond = maxChange_Hz_per_sec;
    rateLim.initialized = false;
    Serial.printf("[FILTER] Rate Limiter initialized: max %.1f Hz/sec\n", maxChange_Hz_per_sec);
}

float applyRateLimiter(float input, unsigned long currentTime) {
    if (!rateLim.initialized) {
        rateLim.lastOutput = input;
        rateLim.lastTime = currentTime;
        rateLim.initialized = true;
        return input;
    }
    
    float dt = (currentTime - rateLim.lastTime) / 1000.0f; // seconds
    if (dt <= 0) return rateLim.lastOutput;
    
    float maxChange = rateLim.maxChangePerSecond * dt;
    float diff = input - rateLim.lastOutput;
    
    float output;
    if (fabs(diff) > maxChange) {
        output = rateLim.lastOutput + (diff > 0 ? maxChange : -maxChange);
    } else {
        output = input;
    }
    
    rateLim.lastOutput = output;
    rateLim.lastTime = currentTime;
    
    return output;
}

// ========== SAMPLE-AND-HOLD VARIABLES ==========
static float lastValidSpeed_kmh = 0;
static float lastValidSpeed_mps = 0;
static float lastValidRPM = 0;
static float lastValidFrequency = 0;
static unsigned long lastValidSpeedTime = 0;
static bool hasValidHistory = false;

// Hold settings
#define MAX_HOLD_TIME_MS 3000
#define HOLD_DECAY_FACTOR 0.995f  // 0.5% decay per second (very subtle)

// ========== DEBUG VARIABLES ==========
SpeedDebugData speedDebug = {0};
static unsigned long lastDebugPrint = 0;
static unsigned long lastUpdateCallTime = 0;
static int updateCallCount = 0;
static int missedUpdates = 0;

// Filter tuning
static float sampleRate = 10.0f;  // 10Hz (100ms interval)

void initSpeedSensor() {
    pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), speedSensorISR, RISING);
    
    // Initialize speed data structure
    speedData.rpm = 0;
    speedData.frequency_hz = 0;
    speedData.speed_kmh = 0;
    speedData.speed_mps = 0;
    speedData.raw_frequency = 0;
    speedData.lastUpdate = 0;
    speedData.valid = false;
    speedData.timeoutMs = 1000;
    
    // Initialize industrial filters
    initLowPassFilter(LPF_CUTOFF_FREQ_HZ, sampleRate);
    initMovingAverage(MOVING_AVERAGE_WINDOW);
    initRateLimiter(50.0f);  // Max 50 Hz change per second
    
    // Initialize sample-and-hold
    lastValidSpeed_kmh = 0;
    lastValidSpeed_mps = 0;
    lastValidRPM = 0;
    lastValidFrequency = 0;
    lastValidSpeedTime = 0;
    hasValidHistory = false;
    
    // Reset pulse counters
    speedPulseCount = 0;
    lastPulseMicros = 0;
    currentPeriodMicros = 0;
    newPulseReady = false;
    
    // Initialize debug
    memset(&speedDebug, 0, sizeof(speedDebug));
    lastDebugPrint = 0;
    lastUpdateCallTime = 0;
    updateCallCount = 0;
    missedUpdates = 0;
    
    Serial.println("[SPEED] Industrial filter system initialized");
}

void IRAM_ATTR speedSensorISR() {
    unsigned long now = micros();
    unsigned long period = now - lastPulseMicros;
    
    speedDebug.isrTriggered = true;
    speedDebug.lastIsrTime = millis();
    
    // Basic sanity check on pulse interval
    if (period > MIN_PULSE_INTERVAL_US && period < MAX_PULSE_INTERVAL_US) {
        currentPeriodMicros = period;
        newPulseReady = true;
        speedPulseCount++;
        speedPulseDetected = true;
        speedDebug.pulseCount++;
    }
    lastPulseMicros = now;
}

void updateSpeed() {
    unsigned long now = millis();
    
    // Track update timing
    updateCallCount++;
    if (lastUpdateCallTime > 0) {
        unsigned long timeSinceLastCall = now - lastUpdateCallTime;
        if (timeSinceLastCall > 150) {
            missedUpdates++;
            Serial.printf("[DEBUG] WARNING: updateSpeed called after %lums (expected 100ms)\n", 
                         timeSinceLastCall);
        }
    }
    lastUpdateCallTime = now;
    speedDebug.lastUpdateTime = now;
    
    // ========== PROCESS NEW PULSE ==========
    if (newPulseReady) {
        newPulseReady = false;
        speedDebug.lastPulseTime = now;
        
        // Step 1: Calculate raw frequency from period
        float rawFrequency = 1000000.0f / (float)currentPeriodMicros;
        speedData.raw_frequency = rawFrequency;
        
        // Store for debugging
        speedDebug.rawFreq[speedDebug.historyIndex % 10] = rawFrequency;
        
        // Step 2: Apply median filter (remove spikes)
        static float medianHistory[10];
        float medianFiltered = applyMedianFilter(medianHistory, MEDIAN_FILTER_WINDOW, rawFrequency);
        
        // Step 3: Apply low-pass filter (remove high-frequency noise)
        float lpfFiltered = applyLowPassFilter(medianFiltered);
        speedDebug.lpfOutput[speedDebug.historyIndex % 10] = lpfFiltered;
        
        // Step 4: Apply moving average (smoothing)
        float movingAvgFiltered = applyMovingAverage(lpfFiltered);
        speedDebug.movingAvgOutput[speedDebug.historyIndex % 10] = movingAvgFiltered;
        
        // Step 5: Apply rate limiter (prevent unrealistic jumps)
        float filteredFrequency = applyRateLimiter(movingAvgFiltered, now);
        speedDebug.filteredFreq[speedDebug.historyIndex % 10] = filteredFrequency;
        speedDebug.historyIndex++;
        
        // Step 6: Calculate RPM and speed
        float rpm = (filteredFrequency * 60.0f) / (float)PULSES_PER_REVOLUTION;
        float wheel_circumference_m = (wheelDiameter_mm / 1000.0f) * 3.14159265359f;
        float speed_mps = (rpm * wheel_circumference_m) / 60.0f;
        float speed_kmh = speed_mps * 3.6f;
        
        // Step 7: Store as last valid values for sample-and-hold
        lastValidSpeed_kmh = speed_kmh;
        lastValidSpeed_mps = speed_mps;
        lastValidRPM = rpm;
        lastValidFrequency = filteredFrequency;
        lastValidSpeedTime = now;
        hasValidHistory = true;
        
        // Step 8: Update global structure
        speedData.frequency_hz = filteredFrequency;
        speedData.rpm = rpm;
        speedData.speed_mps = speed_mps;
        speedData.speed_kmh = speed_kmh;
        speedData.lastUpdate = now;
        speedData.valid = true;
        
        // Debug output
        static unsigned long lastFilterDebug = 0;
        if (now - lastFilterDebug >= 5000) {  // Every 5 seconds
            Serial.printf("[FILTER] Raw: %.1f Hz → Median → LPF(%.1f) → MA(%.1f) → RateLimit(%.1f) → RPM:%.0f → Speed:%.1f km/h\n",
                         rawFrequency, lpfFiltered, movingAvgFiltered, filteredFrequency, rpm, speed_kmh);
            lastFilterDebug = now;
        }
        
        if (now - lastDebugPrint >= 1000) {
            Serial.printf("[SPEED] %.1f km/h | %.1f Hz | RPM:%.0f | Valid\n", 
                         speed_kmh, filteredFrequency, rpm);
            lastDebugPrint = now;
        }
        
    } else {
        // ========== SAMPLE-AND-HOLD: No new pulse ==========
        unsigned long timeSinceLastPulse = 0;
        if (lastValidSpeedTime > 0) {
            timeSinceLastPulse = now - lastValidSpeedTime;
        } else {
            timeSinceLastPulse = MAX_HOLD_TIME_MS + 1000;
        }
        
        if (hasValidHistory && timeSinceLastPulse < MAX_HOLD_TIME_MS) {
            // Hold last valid value with optional decay
            float heldSpeed_kmh = lastValidSpeed_kmh;
            float heldSpeed_mps = lastValidSpeed_mps;
            float heldRPM = lastValidRPM;
            float heldFrequency = lastValidFrequency;
            
            // Apply very subtle decay after 1 second
            if (timeSinceLastPulse > 1000) {
                float decayFactor = pow(HOLD_DECAY_FACTOR, (timeSinceLastPulse - 1000) / 1000.0f);
                heldSpeed_kmh = lastValidSpeed_kmh * decayFactor;
                heldSpeed_mps = lastValidSpeed_mps * decayFactor;
                heldRPM = lastValidRPM * decayFactor;
                heldFrequency = lastValidFrequency * decayFactor;
            }
            
            speedData.speed_kmh = heldSpeed_kmh;
            speedData.speed_mps = heldSpeed_mps;
            speedData.rpm = heldRPM;
            speedData.frequency_hz = heldFrequency;
            speedData.lastUpdate = now;
            speedData.valid = true;
            
            // Periodic hold notification
            static unsigned long lastHoldNotify = 0;
            if (now - lastHoldNotify >= 2000) {
                Serial.printf("[HOLD] No pulse for %.1fs | Speed: %.1f km/h\n",
                             timeSinceLastPulse / 1000.0f, heldSpeed_kmh);
                lastHoldNotify = now;
            }
            
        } else if (speedData.valid) {
            // Timeout - invalidate speed
            speedData.valid = false;
            Serial.printf("[TIMEOUT] No pulse for %.1fs - Speed invalidated\n",
                         timeSinceLastPulse / 1000.0f);
        }
    }
}

// ========== HELPER FUNCTIONS ==========

float getCurrentRPM() {
    return speedData.rpm;
}

float getCurrentFrequency() {
    return speedData.frequency_hz;
}

float getCurrentSpeedKMH() {
    return speedData.speed_kmh;
}

void setWheelDiameter(float diameter_mm) {
    wheelDiameter_mm = diameter_mm;
    Serial.printf("[CONFIG] Wheel diameter set to %.1f mm\n", diameter_mm);
}

void setPulsesPerRevolution(uint8_t pulses) {
    // Note: PULSES_PER_REVOLUTION is a #define, would need modification
    Serial.printf("[CONFIG] Pulses per revolution would be set to %d\n", pulses);
}

void setLowPassFilterCutoff(float cutoffFreq) {
    initLowPassFilter(cutoffFreq, sampleRate);
}

void setMovingAverageWindow(uint8_t windowSize) {
    initMovingAverage(windowSize);
}

// ========== DEBUG FUNCTIONS ==========

void debugSpeedSensor() {
    unsigned long now = millis();
    unsigned long timeSinceLastPulse = 0;
    
    if (lastValidSpeedTime > 0) {
        timeSinceLastPulse = now - lastValidSpeedTime;
    }
    
    Serial.println("\n========== INDUSTRIAL SPEED SENSOR DEBUG ==========");
    Serial.printf("System Status:\n");
    Serial.printf("  Update Calls: %d | Missed Updates: %d\n", updateCallCount, missedUpdates);
    Serial.printf("  Total Pulses: %d | ISR Count: %d\n", 
                 speedDebug.pulseCount, speedDebug.isrTriggered ? 1 : 0);
    
    Serial.printf("\nFilter Configuration:\n");
    Serial.printf("  LPF Cutoff: %.1f Hz | Alpha: %.3f\n", LPF_CUTOFF_FREQ_HZ, lpf.alpha);
    Serial.printf("  Moving Average Window: %d samples\n", movAvg.size);
    Serial.printf("  Median Filter Window: %d points\n", MEDIAN_FILTER_WINDOW);
    Serial.printf("  Rate Limiter: %.0f Hz/sec\n", rateLim.maxChangePerSecond);
    
    Serial.printf("\nCurrent Readings:\n");
    Serial.printf("  Raw: %.1f Hz | Filtered: %.1f Hz\n", 
                 speedData.raw_frequency, speedData.frequency_hz);
    Serial.printf("  Speed: %.1f km/h (%.1f m/s) | RPM: %.0f\n", 
                 speedData.speed_kmh, speedData.speed_mps, speedData.rpm);
    Serial.printf("  Valid: %s | Last Update: %lums ago\n", 
                 speedData.valid ? "YES" : "NO", now - speedData.lastUpdate);
    
    Serial.printf("\nSample-and-Hold:\n");
    Serial.printf("  Last Valid Speed: %.1f km/h\n", lastValidSpeed_kmh);
    Serial.printf("  Time since last pulse: %.1f seconds\n", timeSinceLastPulse / 1000.0f);
    Serial.printf("  Max Hold Time: %.1f seconds\n", MAX_HOLD_TIME_MS / 1000.0f);
    
    Serial.println("\nLast 10 Filtered Readings (Hz):");
    for (int i = 0; i < 10; i++) {
        if (speedDebug.filteredFreq[i] > 0) {
            Serial.printf("  [%d] Raw: %.1f | LPF: %.1f | MA: %.1f | Final: %.1f\n", i,
                         speedDebug.rawFreq[i],
                         speedDebug.lpfOutput[i],
                         speedDebug.movingAvgOutput[i],
                         speedDebug.filteredFreq[i]);
        }
    }
    
    Serial.println("==================================================\n");
}

void resetDebugCounters() {
    speedDebug.pulseCount = 0;
    updateCallCount = 0;
    missedUpdates = 0;
}

float getLastValidSpeed() {
    return lastValidSpeed_kmh;
}

unsigned long getTimeSinceLastPulse() {
    if (lastValidSpeedTime > 0) {
        return millis() - lastValidSpeedTime;
    }
    return MAX_HOLD_TIME_MS + 1000;
}

bool hasRecentValidData(unsigned long maxAgeMs) {
    if (!hasValidHistory) return false;
    return (millis() - lastValidSpeedTime) < maxAgeMs;
}