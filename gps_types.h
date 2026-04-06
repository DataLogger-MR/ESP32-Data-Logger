#ifndef GPS_TYPES_H
#define GPS_TYPES_H

#include <Arduino.h>

// GPS data structure
struct GPSData {
    double latitude;
    double longitude;
    float altitude;
    float speed_kmh;
    float speed_mps;
    float speed_knots;
    float course_deg;
    String cardinal_direction;
    int hour_utc;
    int minute_utc;
    int second_utc;
    int day;
    int month;
    int year;
    int hour_ist;
    int minute_ist;
    int satellites;
    float hdop;
    bool location_valid;
    bool time_valid;
    bool date_valid;
    bool speed_valid;
    bool course_valid;
    bool altitude_valid;
    bool hdop_valid;
    bool satellites_valid;
    unsigned long lastUpdate;
};

// Compass data structure
struct CompassData {
    float heading_deg;
    String cardinal_direction;
    float raw_x;
    float raw_y;
    float raw_z;
    float offset_x;
    float offset_y;
    bool valid;
    unsigned long lastUpdate;
};

// Statistics
struct GPSStatistics {
    unsigned long uptime_ms;
    unsigned long lastFixTime_ms;
    float maxSpeed_kmh;
    float totalDistance_km;
    double lastLat;
    double lastLng;
};

#endif