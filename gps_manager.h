#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPS++.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>        // ADDED for SoftwareSerial
#include <vector>
#include "gps_types.h"
#include "gps_globals.h"
#include "pins.h"

extern SoftwareSerial gpsSerial;   // Changed to SoftwareSerial

void initGPS();
void initCompass();
void calibrateCompass(int duration_seconds = 15);
void updateGPS();
void updateCompass();
void calculateLocalTime();
String getCardinalDirection(float heading);
void formatGPSCSV(char* buffer, size_t bufferSize, unsigned long currentTime);
void resetGPSData();

#endif