#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <WiFiClientSecure.h>      
#include <WebServer.h>
#include <PubSubClient.h>
#include "config.h"
#include "ct_can_sensors.h"

// ================ WIFI GLOBALS ================
extern String wifiStatus;
extern String wifiIP;

// ================ Wi‑Fi station credentials ================
#define WIFI_STA_SSID     "RH-2.4G-8E63D0"
#define WIFI_STA_PASSWORD "44953B8E63D0"

// ================ MQTT Configuration (HiveMQ Cloud) ================
#define MQTT_BROKER       "01792b66dfee4540a546dc894922fb94.s1.eu.hivemq.cloud"
#define MQTT_PORT         8883                    
#define MQTT_TOPIC        "tractor/data"
#define MQTT_CLIENT_ID    "ESP32_Tractor_Logger"
#define MQTT_USERNAME     "MR_TRACTOR"             
#define MQTT_PASSWORD     "#Lokesh000"   

extern WiFiClientSecure espClient;
extern PubSubClient mqttClient;
extern bool mqttReady;
extern CTFlowData ctFlow;
extern CTPressureData ctPressure;
extern CTTemperatureData ctTemp;

// ================ FUNCTION PROTOTYPES ================
void initWiFi();
void initWiFiStation();
void startWebServer();
void handleWebServer();
void createDBCFileIfNeeded();
void sendMQTTData();
void sendInfluxDBData();  

void handleRoot();
void handleLiveData();
void handleFileList();
void handleCommand();
void handleFileDownload();
void handleNotFound();
String listFilesHTML(File dir, String path);
String listFilesText(File dir, String path);
String getWebPage();

void handleDBCUpload();
void handleDBCParse();
void handleDBCSave();
void handleDBCStatus();
void handleDBCDelete();

void handleI2CGetConfig();
void handleI2CSaveConfig();
void handleI2CConfigPage();
void createI2CConfigHTML();

#endif