#include "gps_manager.h"
#include "globals.h"
#include "session_manager.h"
#include "file_manager.h"
#include "utils.h"
#include <math.h>

// ================ GLOBALS ================
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Magnetic declination for Bangalore (adjust as needed)
float declinationAngle = 0.55;  // degrees

// Calibration offsets
float magOffsetX = 0;
float magOffsetY = 0;

// ================ INITIALIZATION ================
void initGPS() {
    
    // Use runtime GPS baud rate
    gpsSerial.begin(gpsBaudRate, SERIAL_8N1, 16, 17);
    delay(100);
    
    // Clear any pending data
    while (gpsSerial.available()) {
        gpsSerial.read();
    }
    
    gpsInitialized = true;
    
    // Reset GPS data structure
    resetGPSData();
    
}

void resetGPSData() {
    memset(&gpsData, 0, sizeof(GPSData));
    gpsData.lastUpdate = 0;
    gpsData.location_valid = false;
    gpsData.time_valid = false;
    gpsData.date_valid = false;
    gpsData.speed_valid = false;
    gpsData.course_valid = false;
    gpsData.altitude_valid = false;
    gpsData.hdop_valid = false;
    gpsData.satellites_valid = false;
    
    // Reset statistics
    gpsStats.uptime_ms = millis();
    gpsStats.lastFixTime_ms = 0;
    gpsStats.maxSpeed_kmh = 0;
    gpsStats.totalDistance_km = 0;
    gpsStats.lastLat = 0;
    gpsStats.lastLng = 0;
    
}

void initCompass() {
    Serial.print("🧭 Initializing compass... ");
    
    Wire.begin();
    
    if (!mag.begin()) {
        Serial.println("❌ NOT FOUND!");
        compassInitialized = false;
        return;
    }
    
    compassInitialized = true;
    
    // Display sensor info
    sensor_t sensor;
    mag.getSensor(&sensor);
    Serial.printf("✅ %s found\n", sensor.name);
    
    // Initialize with zero offsets (calibration needed)
    compassData.offset_x = 0;
    compassData.offset_y = 0;
    compassData.valid = false;
}

void calibrateCompass(int duration_seconds) {
    if (!compassInitialized) {
        Serial.println("❌ Compass not initialized");
        return;
    }
    
    Serial.println("\n=== COMPASS CALIBRATION ===");
    Serial.println("Rotate device in figure-8 pattern for " + String(duration_seconds) + " seconds");
    
    float minX = 1000, maxX = -1000, minY = 1000, maxY = -1000;
    
    unsigned long startTime = millis();
    unsigned long lastPrint = 0;
    
    while (millis() - startTime < duration_seconds * 1000UL) {
        sensors_event_t event;
        mag.getEvent(&event);
        
        minX = min(minX, event.magnetic.x);
        maxX = max(maxX, event.magnetic.x);
        minY = min(minY, event.magnetic.y);
        maxY = max(maxY, event.magnetic.y);
        
        if (millis() - lastPrint > 1000) {
            Serial.print(".");
            lastPrint = millis();
        }
        delay(50);
    }
    
    Serial.println("\n\n✅ Calibration complete!");
    Serial.print("X range: "); Serial.print(minX); Serial.print(" to "); Serial.println(maxX);
    Serial.print("Y range: "); Serial.print(minY); Serial.print(" to "); Serial.println(maxY);
    
    magOffsetX = (maxX + minX) / 2;
    magOffsetY = (maxY + minY) / 2;
    
    compassData.offset_x = magOffsetX;
    compassData.offset_y = magOffsetY;
    
    Serial.print("Set offsets: magOffsetX = "); Serial.print(magOffsetX);
    Serial.print(", magOffsetY = "); Serial.println(magOffsetY);
}

// ================ UPDATE FUNCTIONS ================
void updateGPS() {
    if (!gpsInitialized) return;
    
    unsigned long now = millis();
    static unsigned long lastValidPrint = 0;
    
    // Read all available GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    
    // Update GPS data structure
    gpsData.location_valid = gps.location.isValid();
    gpsData.time_valid = gps.time.isValid();
    gpsData.date_valid = gps.date.isValid();
    gpsData.speed_valid = gps.speed.isValid();
    gpsData.course_valid = gps.course.isValid();
    gpsData.altitude_valid = gps.altitude.isValid();
    gpsData.hdop_valid = gps.hdop.isValid();
    gpsData.satellites_valid = gps.satellites.isValid();
    
    if (gpsData.location_valid) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
        gpsData.lastUpdate = now;
        
        // Update statistics
        if (gps.speed.kmph() > gpsStats.maxSpeed_kmh) {
            gpsStats.maxSpeed_kmh = gps.speed.kmph();
        }
        
        // Calculate distance if moving
        if (gpsData.speed_valid && gps.speed.kmph() > 1.0) {
            if (gpsStats.lastLat != 0 && gpsStats.lastLng != 0) {
                gpsStats.totalDistance_km += TinyGPSPlus::distanceBetween(
                    gpsStats.lastLat, gpsStats.lastLng,
                    gpsData.latitude, gpsData.longitude
                ) / 1000.0;  // Convert meters to km
            }
            gpsStats.lastLat = gpsData.latitude;
            gpsStats.lastLng = gpsData.longitude;
        }
        
        if (gps.location.age() < 1000) {
            gpsStats.lastFixTime_ms = now;
        }
    }
    
    if (gpsData.altitude_valid) {
        gpsData.altitude = gps.altitude.meters();
    }
    
    if (gpsData.speed_valid) {
        gpsData.speed_kmh = gps.speed.kmph();
        gpsData.speed_mps = gps.speed.mps();
        gpsData.speed_knots = gps.speed.knots();
    }
    
    if (gpsData.course_valid) {
        gpsData.course_deg = gps.course.deg();
        gpsData.cardinal_direction = getCardinalDirection(gpsData.course_deg);
    }
    
    if (gpsData.time_valid) {
        gpsData.hour_utc = gps.time.hour();
        gpsData.minute_utc = gps.time.minute();
        gpsData.second_utc = gps.time.second();
        calculateLocalTime();  // Calculate IST
    }
    
    if (gpsData.date_valid) {
        gpsData.day = gps.date.day();
        gpsData.month = gps.date.month();
        gpsData.year = gps.date.year();
    }
    
    if (gpsData.hdop_valid) {
        gpsData.hdop = gps.hdop.hdop();
    }
    
    if (gpsData.satellites_valid) {
        gpsData.satellites = gps.satellites.value();
    }
    
    // Print valid data status periodically
    if (now - lastValidPrint > 30000) { // Every 30 seconds
        if (gpsData.location_valid || gpsData.time_valid) {
            Serial.println("✅ GPS has valid data");
            if (gpsData.location_valid) {
                Serial.printf("  Location: %.6f, %.6f\n", gpsData.latitude, gpsData.longitude);
            }
            if (gpsData.time_valid) {
                Serial.printf("  UTC Time: %02d:%02d:%02d\n", 
                             gpsData.hour_utc, gpsData.minute_utc, gpsData.second_utc);
            }
        } else {
            Serial.println("⏳ GPS waiting for fix...");
        }
        lastValidPrint = now;
    }
    
    // --- MODIFIED: Only set uartDataPresent, do NOT start a session ---
    if (gpsData.location_valid || gpsData.time_valid) {
        uartDataPresent = true;
        lastUARTActivity = now;
    }
    // ----------------------------------------------------------------
}

void updateCompass() {
    if (!compassInitialized) return;
    
    sensors_event_t event;
    mag.getEvent(&event);
    
    // Store raw data
    compassData.raw_x = event.magnetic.x;
    compassData.raw_y = event.magnetic.y;
    compassData.raw_z = event.magnetic.z;
    
    // Apply calibration offsets
    float x = event.magnetic.x - magOffsetX;
    float y = event.magnetic.y - magOffsetY;
    
    // Calculate heading
    float heading = atan2(y, x);
    heading += declinationAngle * (M_PI / 180.0);
    
    if (heading < 0) heading += 2 * M_PI;
    if (heading > 2 * M_PI) heading -= 2 * M_PI;
    
    compassData.heading_deg = heading * 180.0 / M_PI;
    compassData.cardinal_direction = getCardinalDirection(compassData.heading_deg);
    compassData.valid = true;
    compassData.lastUpdate = millis();
}

void calculateLocalTime() {
    if (!gpsData.time_valid) return;
    
    // IST = UTC + 5:30
    int localHour = gpsData.hour_utc + 5;
    int localMinute = gpsData.minute_utc + 30;
    
    if (localMinute >= 60) {
        localMinute -= 60;
        localHour++;
    }
    if (localHour >= 24) {
        localHour -= 24;
    }
    
    gpsData.hour_ist = localHour;
    gpsData.minute_ist = localMinute;
}

String getCardinalDirection(float heading) {
    const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                                 "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int index = (int)((heading + 11.25) / 22.5) % 16;
    return String(directions[index]);
}

// ================ CSV FORMATTING ================
void formatGPSCSV(char* buffer, size_t bufferSize, unsigned long currentTime) {
    int pos = 0;
    
    // GPS Data
    if (gpsData.location_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.6f,%.6f", 
                        gpsData.latitude, gpsData.longitude);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,");
    }
    
    if (gpsData.altitude_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f", gpsData.altitude);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (gpsData.speed_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%.1f,%.1f", 
                        gpsData.speed_kmh, gpsData.speed_mps, gpsData.speed_knots);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,");
    }
    
    if (gpsData.course_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%s", 
                        gpsData.course_deg, gpsData.cardinal_direction.c_str());
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,");
    }
    
    // Time (UTC)
    if (gpsData.time_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%02d:%02d:%02d",
                        gpsData.hour_utc, gpsData.minute_utc, gpsData.second_utc);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Date
    if (gpsData.date_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%04d-%02d-%02d",
                        gpsData.year, gpsData.month, gpsData.day);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Time (IST)
    if (gpsData.time_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%02d:%02d:%02d",
                        gpsData.hour_ist, gpsData.minute_ist, gpsData.second_utc);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Quality
    if (gpsData.satellites_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%d", gpsData.satellites);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    if (gpsData.hdop_valid) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f", gpsData.hdop);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",");
    }
    
    // Compass
    if (compassData.valid && isValid(compassData.lastUpdate, 5000, currentTime)) {
        pos += snprintf(buffer + pos, bufferSize - pos, ",%.1f,%s,%.1f,%.1f,%.1f",
                        compassData.heading_deg, compassData.cardinal_direction.c_str(),
                        compassData.raw_x, compassData.raw_y, compassData.raw_z);
    } else {
        pos += snprintf(buffer + pos, bufferSize - pos, ",,,,,");
    }
    
    // Statistics
    unsigned long uptime_seconds = (currentTime - gpsStats.uptime_ms) / 1000;
    pos += snprintf(buffer + pos, bufferSize - pos, ",%lu,%.1f,%.3f",
                    uptime_seconds, gpsStats.maxSpeed_kmh, gpsStats.totalDistance_km);
}