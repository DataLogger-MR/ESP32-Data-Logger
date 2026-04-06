#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <WiFiClientSecure.h>      // For TLS connection
#include <WebServer.h>
#include <PubSubClient.h>
#include "config.h"

// ================ WIFI GLOBALS ================
extern String wifiStatus;
extern String wifiIP;

// ================ Wi‑Fi station credentials ================
#define WIFI_STA_SSID     "Moonrider_00"
#define WIFI_STA_PASSWORD "Moonrider@567"

// ================ MQTT Configuration (HiveMQ Cloud) ================
#define MQTT_BROKER       "01792b66dfee4540a546dc894922fb94.s1.eu.hivemq.cloud"
#define MQTT_PORT         8883                     // TLS port
#define MQTT_TOPIC        "tractor/data"
#define MQTT_CLIENT_ID    "ESP32_Tractor_Logger"
#define MQTT_USERNAME     "MR_TRACTOR"             // Replace with your actual username
#define MQTT_PASSWORD     "#Lokesh000"   // Replace with the password you set

// Use WiFiClientSecure for TLS
extern WiFiClientSecure espClient;
extern PubSubClient mqttClient;
extern bool mqttReady;

// ================ FUNCTION PROTOTYPES ================
void initWiFi();
void initWiFiStation();
void startWebServer();
void handleWebServer();
void createDBCFileIfNeeded();
void sendMQTTData();
void sendInfluxDBData();   // Keep for compatibility, will call MQTT

// Web server handlers
void handleRoot();
void handleLiveData();
void handleFileList();
void handleCommand();
void handleFileDownload();
void handleNotFound();
String listFilesHTML(File dir, String path);
String listFilesText(File dir, String path);
String getWebPage();

// DBC handlers
void handleDBCUpload();
void handleDBCParse();
void handleDBCSave();
void handleDBCStatus();
void handleDBCDelete();

// I2C config handlers
void handleI2CGetConfig();
void handleI2CSaveConfig();
void handleI2CConfigPage();
void createI2CConfigHTML();

#endif