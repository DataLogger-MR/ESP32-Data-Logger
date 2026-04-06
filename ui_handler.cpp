#include "ui_handler.h"
#include "sd_card.h"
#include "session_manager.h"
#include "ecu_state.h"
#include "data_logger.h"
#include "file_manager.h"
#include "can_decoder.h"
#include "utils.h"
#include "globals.h"
#include "gps_globals.h"
#include "file_manager.h"
#include <SPIFFS.h>

char uiCommandBuffer[128];
int uiCommandIndex = 0;

// External references
extern bool loggingActive;
extern unsigned long lastStatsTime;
extern SessionState_t sessionState;
extern ECUState_t ecuState;
extern uint32_t currentSessionId;
extern uint32_t currentFileSequence;
extern uint32_t sessionRecordCounter;
extern uint32_t fileRecordCounter;
extern char currentFilePath[128];
extern unsigned long currentFileSize;
extern unsigned long messageCount;
extern unsigned long acceptedCount;
extern unsigned long filteredOutCount;
extern unsigned long loggedCount;
extern unsigned long startTime;
extern bool sdReady;
extern char sessionLogPath[];
extern uint32_t lastRecError;
extern uint32_t lastTecError;

// Add these with other extern declarations
extern int logIntervalMs;
extern int maxFileSizeMB;
extern bool rotateHourlyEnabled;
extern int autoDeleteDays;
extern int gpsBaudRate;
extern int gpsUpdateInterval;
extern String wifiSSID;
extern String wifiPassword;
extern int bufferSize;
extern int ecuTimeout;
extern void saveConfigToSPIFFS();  // Add this function declaration

// Function declarations from other modules
extern void deleteFile(const char* fileName);
extern void createTestFile(const char* fileName);
extern void listLogsByDate(int year, int month, int day);
extern void rotateFile(RotateReason_t reason);
extern void resetStatistics();
extern void sendFile(const char* fileName);
extern void listFiles();
extern void listDiagFiles();
extern void createDisconnectDiagFile(RotateReason_t reason, uint32_t lastRecErr, uint32_t lastTecErr);

void initUI() {
  Serial2.begin(921600, SERIAL_8N1, UI_RXD2, UI_TXD2);
  while(Serial2.available()) Serial2.read();
  
  sendToUILn("BMS_LOGGER_READY");
  delay(10);
  sendToUILn(sdReady ? "SD_OK" : "SD_ERROR");
  delay(10);
}

void processUICommands() {
  if (!Serial2.available()) return;
  
  char c = Serial2.read();
  Serial.print(c);
  
  if (c == '\n') {
    if (uiCommandIndex > 0) {
      uiCommandBuffer[uiCommandIndex] = '\0';
      Serial.println();
      processUICommand(uiCommandBuffer);
      uiCommandIndex = 0;
    }
  } else if (c != '\r' && uiCommandIndex < sizeof(uiCommandBuffer) - 1) {
    uiCommandBuffer[uiCommandIndex++] = c;
  }
}

void processConfigCommand(String cmd) {
    cmd.trim();
    bool configChanged = false;
    
    // Debug: Print the command being processed
    Serial.print("Processing config command: ");
    Serial.println(cmd);
    
    // ================ LOGGING SETTINGS ================
    if (cmd.startsWith("config logging interval")) {
        int interval = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (interval >= 10 && interval <= 10000) {
            logIntervalMs = interval;
            Serial.printf("Logging interval set to %d ms\n", interval);
            sendToUILn("CONFIG_OK: logging interval updated");
            configChanged = true;
            applyConfigurationChanges();
        } else {
            sendToUILn("CONFIG_ERROR: Invalid interval value (10-10000)");
        }
    }
    else if (cmd.startsWith("config logging maxsize")) {
        int maxSize = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (maxSize >= 1 && maxSize <= 1000) {
            maxFileSizeMB = maxSize;
            Serial.printf("Max file size set to %d MB\n", maxSize);
            sendToUILn("CONFIG_OK: max file size updated");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Invalid max size (1-1000)");
        }
    }
    else if (cmd.startsWith("config logging autodelete")) {
        int days = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (days >= 0 && days <= 365) {
            autoDeleteDays = days;
            Serial.printf("Auto delete days set to %d\n", days);
            sendToUILn("CONFIG_OK: auto delete days updated");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Invalid days (0-365)");
        }
    }
    else if (cmd.startsWith("config logging rotatehourly")) {
        int rotateHourly = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        rotateHourlyEnabled = (rotateHourly == 1);
        Serial.printf("Hourly rotation: %s\n", rotateHourlyEnabled ? "ENABLED" : "DISABLED");
        sendToUILn("CONFIG_OK: hourly rotation updated");
        configChanged = true;
    }
    else if (cmd.startsWith("config logging includedate")) {
        int includeDate = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        // Note: INCLUDE_DATE_IN_FILENAME is a #define, so this is informational only
        Serial.printf("Include date in filename: %s (requires restart)\n", includeDate ? "ENABLED" : "DISABLED");
        sendToUILn("CONFIG_WARNING: Include date change requires restart");
        configChanged = true;
    }
    else if (cmd.startsWith("config logging printcanmsgs")) {
        int printMsgs = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        // Note: PRINT_EVERY_MESSAGE is a #define, so this is informational only
        Serial.printf("Print CAN messages: %s (requires recompile)\n", printMsgs ? "ENABLED" : "DISABLED");
        sendToUILn("CONFIG_WARNING: Print CAN messages requires recompile");
        configChanged = true;
    }
    else if (cmd.startsWith("config logging printlogged")) {
        int printLogged = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        // Note: PRINT_LOGGED_DATA is a #define, so this is informational only
        Serial.printf("Print logged data: %s (requires recompile)\n", printLogged ? "ENABLED" : "DISABLED");
        sendToUILn("CONFIG_WARNING: Print logged data requires recompile");
        configChanged = true;
    }
    
    // ================ CAN BUS SETTINGS ================
    else if (cmd.startsWith("config can filtermode")) {
        int mode = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        Serial.printf("Filter mode changed to %d (requires restart)\n", mode);
        sendToUILn("CONFIG_WARNING: Filter mode change requires restart");
        configChanged = true;
    }
    else if (cmd.startsWith("config can baud")) {
        int baud = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        canBaudRate = baud;
        Serial.printf("CAN baud rate changed to %d kbps (requires restart)\n", baud);
        sendToUILn("CONFIG_WARNING: CAN baud change requires restart");
        configChanged = true;
    }
    else if (cmd.startsWith("config can rxqueue")) {
        int queueSize = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        canRxQueueSize = queueSize;
        Serial.printf("CAN RX queue size changed to %d (requires restart)\n", queueSize);
        sendToUILn("CONFIG_WARNING: CAN queue size change requires restart");
        configChanged = true;
    }
    
    // ================ WIFI SETTINGS ================
    else if (cmd.startsWith("config wifi ssid")) {
        String ssid = cmd.substring(cmd.indexOf(' ', 15) + 1);
        ssid.trim();
        // Remove quotes if present
        if (ssid.startsWith("\"") && ssid.endsWith("\"")) {
            ssid = ssid.substring(1, ssid.length() - 1);
        }
        if (ssid.length() > 0 && ssid.length() <= 32) {
            wifiSSID = ssid;
            Serial.printf("WiFi SSID set to %s\n", ssid.c_str());
            sendToUILn("CONFIG_OK: WiFi SSID updated");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Invalid SSID (1-32 chars)");
        }
    }
    else if (cmd.startsWith("config wifi password")) {
        String password = cmd.substring(cmd.indexOf(' ', 15) + 1);
        password.trim();
        // Remove quotes if present
        if (password.startsWith("\"") && password.endsWith("\"")) {
            password = password.substring(1, password.length() - 1);
        }
        if (password.length() >= 8 && password.length() <= 63) {
            wifiPassword = password;
            Serial.printf("WiFi password updated\n");
            sendToUILn("CONFIG_OK: WiFi password updated");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Password must be 8-63 chars");
        }
    }
    else if (cmd.startsWith("config wifi apssid")) {
        // AP SSID is defined in config.cpp, this is informational only
        String apSsid = cmd.substring(cmd.indexOf(' ', 15) + 1);
        apSsid.trim();
        Serial.printf("AP SSID would be set to %s (requires recompile)\n", apSsid.c_str());
        sendToUILn("CONFIG_WARNING: AP SSID change requires recompile");
        configChanged = true;
    }
    else if (cmd.startsWith("config wifi appassword")) {
        // AP Password is defined in config.cpp, this is informational only
        String apPassword = cmd.substring(cmd.indexOf(' ', 15) + 1);
        apPassword.trim();
        Serial.printf("AP Password would be set (requires recompile)\n");
        sendToUILn("CONFIG_WARNING: AP Password change requires recompile");
        configChanged = true;
    }
    
    // ================ GPS SETTINGS ================
    else if (cmd.startsWith("config gps baud")) {
        int baud = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (baud == 4800 || baud == 9600 || baud == 19200 || baud == 38400 || baud == 57600 || baud == 115200) {
            gpsBaudRate = baud;
            Serial.printf("GPS baud rate set to %d\n", baud);
            sendToUILn("CONFIG_OK: GPS baud rate updated (restart GPS to apply)");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Invalid baud rate");
        }
    }
    else if (cmd.startsWith("config gps interval")) {
        int interval = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (interval >= 100 && interval <= 10000) {
            gpsUpdateInterval = interval;
            Serial.printf("GPS update interval set to %d ms\n", interval);
            sendToUILn("CONFIG_OK: GPS interval updated");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Invalid interval (100-10000)");
        }
    }
    
    // ================ MQTT SETTINGS ================
    else if (cmd.startsWith("config mqtt broker")) {
        String broker = cmd.substring(cmd.indexOf(' ', 15) + 1);
        broker.trim();
        if (broker.startsWith("\"") && broker.endsWith("\"")) {
            broker = broker.substring(1, broker.length() - 1);
        }
        mqttBroker = broker;
        Serial.printf("MQTT broker set to %s\n", broker.c_str());
        sendToUILn("CONFIG_OK: MQTT broker updated (requires restart)");
        configChanged = true;
    }
    else if (cmd.startsWith("config mqtt port")) {
        int port = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (port >= 1 && port <= 65535) {
            Serial.printf("MQTT port set to %d (requires restart)\n", port);
            sendToUILn("CONFIG_WARNING: MQTT port change requires restart");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Invalid port (1-65535)");
        }
    }
    else if (cmd.startsWith("config mqtt topic")) {
        String topic = cmd.substring(cmd.indexOf(' ', 15) + 1);
        topic.trim();
        // Remove quotes if present
        if (topic.startsWith("\"") && topic.endsWith("\"")) {
            topic = topic.substring(1, topic.length() - 1);
        }
        Serial.printf("MQTT topic set to %s (requires restart)\n", topic.c_str());
        sendToUILn("CONFIG_WARNING: MQTT topic change requires restart");
        configChanged = true;
    }
    else if (cmd.startsWith("config mqtt clientid")) {
        String clientId = cmd.substring(cmd.indexOf(' ', 15) + 1);
        clientId.trim();
        // Remove quotes if present
        if (clientId.startsWith("\"") && clientId.endsWith("\"")) {
            clientId = clientId.substring(1, clientId.length() - 1);
        }
        Serial.printf("MQTT client ID set to %s (requires restart)\n", clientId.c_str());
        sendToUILn("CONFIG_WARNING: MQTT client ID change requires restart");
        configChanged = true;
    }
    else if (cmd.startsWith("config mqtt username")) {
        String username = cmd.substring(cmd.indexOf(' ', 15) + 1);
        username.trim();
        // Remove quotes if present
        if (username.startsWith("\"") && username.endsWith("\"")) {
            username = username.substring(1, username.length() - 1);
        }
        Serial.printf("MQTT username set to %s (requires restart)\n", username.c_str());
        sendToUILn("CONFIG_WARNING: MQTT username change requires restart");
        configChanged = true;
    }
    else if (cmd.startsWith("config mqtt password")) {
        String password = cmd.substring(cmd.indexOf(' ', 15) + 1);
        password.trim();
        // Remove quotes if present
        if (password.startsWith("\"") && password.endsWith("\"")) {
            password = password.substring(1, password.length() - 1);
        }
        Serial.printf("MQTT password updated (requires restart)\n");
        sendToUILn("CONFIG_WARNING: MQTT password change requires restart");
        configChanged = true;
    }
    
    // ================ SYSTEM SETTINGS ================
    else if (cmd.startsWith("config system debug")) {
        int debug = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        #ifdef ENABLE_SERIAL_DEBUG
            Serial.printf("Debug output %s (requires restart)\n", debug ? "enabled" : "disabled");
            sendToUILn("CONFIG_WARNING: Debug change requires restart");
        #else
            sendToUILn("CONFIG_WARNING: Debug not compiled in");
        #endif
        configChanged = true;
    }
    else if (cmd.startsWith("config system buffersize")) {
        int size = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (size >= 1024 && size <= 65536) {
            bufferSize = size;
            Serial.printf("Buffer size set to %d bytes\n", size);
            sendToUILn("CONFIG_OK: Buffer size updated (restart to apply)");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Buffer size must be 1024-65536");
        }
    }
    else if (cmd.startsWith("config system ecutimeout")) {
        int timeout = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        if (timeout >= 5000 && timeout <= 60000) {
            ecuTimeout = timeout;
            Serial.printf("ECU timeout set to %d ms\n", timeout);
            sendToUILn("CONFIG_OK: ECU timeout updated");
            configChanged = true;
        } else {
            sendToUILn("CONFIG_ERROR: Timeout must be 5000-60000");
        }
    }
    else if (cmd.startsWith("config system flushinterval")) {
        int flushInterval = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        Serial.printf("Flush interval set to %d ms (requires restart)\n", flushInterval);
        sendToUILn("CONFIG_WARNING: Flush interval change requires restart");
        configChanged = true;
    }
    else if (cmd.startsWith("config system maxfiles")) {
        int maxFiles = cmd.substring(cmd.lastIndexOf(' ')).toInt();
        Serial.printf("Max files set to %d (requires restart)\n", maxFiles);
        sendToUILn("CONFIG_WARNING: Max files change requires restart");
        configChanged = true;
    }
    else {
        // Unknown command - log it for debugging
        Serial.printf("Unknown config command: %s\n", cmd.c_str());
        sendToUILn("CONFIG_ERROR: Unknown configuration command");
        return;
    }
    
    // Save to SPIFFS if any config changed
    if (configChanged) {
        saveConfigToSPIFFS();
        sendToUILn("CONFIG_SAVED_TO_SPIFFS");
        
        // Print current config to verify
        Serial.println("\n=== CONFIGURATION SAVED ===");
        Serial.printf("logIntervalMs: %d\n", logIntervalMs);
        Serial.printf("maxFileSizeMB: %d\n", maxFileSizeMB);
        Serial.printf("rotateHourlyEnabled: %d\n", rotateHourlyEnabled);
        Serial.printf("autoDeleteDays: %d\n", autoDeleteDays);
        Serial.printf("gpsBaudRate: %d\n", gpsBaudRate);
        Serial.printf("gpsUpdateInterval: %d\n", gpsUpdateInterval);
        Serial.printf("wifiSSID: %s\n", wifiSSID.c_str());
        Serial.printf("wifiPassword: %s\n", wifiPassword.c_str());
        Serial.printf("bufferSize: %d\n", bufferSize);
        Serial.printf("ecuTimeout: %d\n", ecuTimeout);
        Serial.println("===========================\n");
    }
}


void processUICommand(char* cmd) {
  Serial.print("⚙️ UI Command: \"");
  Serial.print(cmd);
  Serial.println("\"");
  
  // Handle batch commands (multiple lines separated by newlines)
  if (strchr(cmd, '\n') != NULL) {
    char* line = strtok(cmd, "\n");
    while (line != NULL) {
      // Trim whitespace
      while (*line == ' ') line++;
      if (strlen(line) > 0) {
        Serial.printf("Processing batch command: %s\n", line);
        processUICommand(line);
      }
      line = strtok(NULL, "\n");
    }
    return;
  }
  
  // Trim whitespace
  while (*cmd == ' ') cmd++;
  
  // Check if this is a config command FIRST (before splitting)
  if (strncmp(cmd, "config", 6) == 0) {
    processConfigCommand(String(cmd));
    return;
  }
  
  char* space = strchr(cmd, ' ');
  
  if (space != NULL) {
    *space = '\0';
    char* param = space + 1;
    while (*param == ' ') param++;
    
    if (strcmp(cmd, "read") == 0 || strcmp(cmd, "send") == 0) {
      sendFile(param);
    } else if (strcmp(cmd, "delete") == 0) {
      deleteFile(param);
    } else if (strcmp(cmd, "create") == 0) {
      createTestFile(param);
    } else if (strcmp(cmd, "loglist") == 0) {
      handleFileManagementCommands(cmd, param);
    } else if (strcmp(cmd, "send") == 0) {
      sendFile(param);
    } else {
      sendToUILn("ERROR: Unknown command");
    }
  } else {
    // Handle commands without spaces (list, status, etc.)
    if (strcmp(cmd, "list") == 0) {
      listFiles();
    } else if (strcmp(cmd, "listdiag") == 0) {
      listDiagFiles();
    } else if (strcmp(cmd, "sessions") == 0) {           // NEW: Session history command
      sendSessionHistory();
    } else if (strcmp(cmd, "info") == 0) {
      sendCardInfo();
    } else if (strcmp(cmd, "status") == 0) {
      sendStatus();
    } else if (strcmp(cmd, "live") == 0) {
      sendLiveData();
    } else if (strcmp(cmd, "stats") == 0) {
      sendStats();
    } else if (strcmp(cmd, "logstart") == 0) {
      loggingActive = true;
      if (sessionState == SESSION_STATE_WAITING) {
        sessionState = SESSION_STATE_ACTIVE;
      }
      sendToUILn("LOGGING_STARTED");
    } else if (strcmp(cmd, "logstop") == 0) {
      loggingActive = false;
      sessionState = SESSION_STATE_WAITING;
      sendToUILn("LOGGING_STOPPED");
    } else if (strcmp(cmd, "reset") == 0) {
      resetStatistics();
    } else if (strcmp(cmd, "help") == 0) {
      sendHelp();
    } else if (strcmp(cmd, "logstatus") == 0 || 
               strcmp(cmd, "logrotate") == 0 ||
               strcmp(cmd, "logsummary") == 0) {
      handleFileManagementCommands(cmd, NULL);
    } else if (strcmp(cmd, "creatediag") == 0) {
      handleDiagCommands(cmd, NULL);
    } else if (strcmp(cmd, "gpsinfo") == 0) {
      sendGPSInfo();
    } else if (strcmp(cmd, "showconfig") == 0) {
      // Show current configuration
      sendToUILn("CURRENT_CONFIG_BEGIN");
      sendToUI("log_interval_ms: "); sendToUILn(String(logIntervalMs).c_str());
      sendToUI("max_file_size_mb: "); sendToUILn(String(maxFileSizeMB).c_str());
      sendToUI("rotate_hourly: "); sendToUILn(rotateHourlyEnabled ? "1" : "0");
      sendToUI("auto_delete_days: "); sendToUILn(String(autoDeleteDays).c_str());
      sendToUI("gps_baud_rate: "); sendToUILn(String(gpsBaudRate).c_str());
      sendToUI("gps_update_interval: "); sendToUILn(String(gpsUpdateInterval).c_str());
      sendToUI("wifi_ssid: "); sendToUILn(wifiSSID.c_str());
      sendToUI("wifi_password: "); sendToUILn(wifiPassword.c_str());
      sendToUI("buffer_size: "); sendToUILn(String(bufferSize).c_str());
      sendToUI("ecu_timeout: "); sendToUILn(String(ecuTimeout).c_str());
      sendToUILn("CURRENT_CONFIG_END");
    } else if (strcmp(cmd, "saveconfig") == 0) {
      saveConfigToSPIFFS();
      sendToUILn("CONFIG_FORCE_SAVED");
    } else if (strcmp(cmd, "testconfig") == 0) {
      logIntervalMs = 50;
      maxFileSizeMB = 50;
      rotateHourlyEnabled = true;
      autoDeleteDays = 15;
      gpsBaudRate = 9600;
      gpsUpdateInterval = 200;
      wifiSSID = "Test_Network";
      wifiPassword = "test123";
      bufferSize = 8192;
      ecuTimeout = 15000;
      saveConfigToSPIFFS();
      sendToUILn("TEST_CONFIG_SAVED");
      sendToUILn("Please reboot to verify persistence");
    } else if (strcmp(cmd, "ping") == 0) {
      sendToUILn("pong");
    } else if (strcmp(cmd, "listspiffs") == 0) {
      if (!SPIFFS.begin(true)) {
        sendToUILn("SPIFFS_ERROR: Failed to mount");
        return;
      }
      sendToUILn("SPIFFS_LIST_BEGIN");
      File root = SPIFFS.open("/");
      if (root) {
        File file = root.openNextFile();
        while (file) {
          sendToUI(file.name());
          sendToUI(" (");
          sendToUI(String(file.size()).c_str());
          sendToUILn(" bytes)");
          file = root.openNextFile();
        }
        root.close();
      } else {
        sendToUILn("SPIFFS_ERROR: Cannot open root");
      }
      sendToUILn("SPIFFS_LIST_END");
    } else {
      sendToUI("UNKNOWN: ");
      sendToUILn(cmd);
    }
  }
}

void sendToUI(const char* message) {
  Serial2.print(message);
}

void sendToUILn(const char* message) {
  Serial2.println(message);
}

void handleFileManagementCommands(char* cmd, char* param) {
  if (strcmp(cmd, "logstatus") == 0) {
    sendToUILn("LOG_STATUS_BEGIN");
    sendToUI("Current file: "); sendToUILn(currentFilePath);
    sendToUI("File size: "); sendToUI(String(currentFileSize).c_str()); sendToUILn(" bytes");
    sendToUI("Session ID: "); sendToUILn(String(currentSessionId).c_str());
    sendToUI("File sequence: "); sendToUILn(String(currentFileSequence).c_str());
    sendToUI("Session records: "); sendToUILn(String(sessionRecordCounter).c_str());
    sendToUI("File records: "); sendToUILn(String(fileRecordCounter).c_str());
    sendToUI("ECU State: "); sendToUILn(String(ecuState).c_str());
    sendToUI("Session State: "); sendToUILn(String(sessionState).c_str());
    sendToUILn("LOG_STATUS_END");
  }
  else if (strcmp(cmd, "logrotate") == 0) {
    rotateFile(ROTATE_REASON_USER_COMMAND);
    sendToUILn("Log rotated");
  }
  else if (strcmp(cmd, "loglist") == 0) {
    if (param) {
      int year, month, day;
      if (sscanf(param, "%d-%d-%d", &year, &month, &day) == 3) {
        listLogsByDate(year, month, day);
      } else {
        sendToUILn("Usage: loglist YYYY-MM-DD");
      }
    } else {
      sendToUILn("Usage: loglist YYYY-MM-DD");
    }
  }
  else if (strcmp(cmd, "logsummary") == 0) {
    File file = SD.open(sessionLogPath);
    if (file) {
      sendToUILn("SESSION_SUMMARY_BEGIN");
      while (file.available()) {
        String line = file.readStringUntil('\n');
        if (line.length() > 0 && !line.startsWith("SessionID")) {
          sendToUILn(line.c_str());
        }
      }
      file.close();
      sendToUILn("SESSION_SUMMARY_END");
    }
  }
}

void handleDiagCommands(char* cmd, char* param) {
  if (strcmp(cmd, "listdiag") == 0) {
    listDiagFiles();
  }
  else if (strcmp(cmd, "creatediag") == 0) {
    createDisconnectDiagFile(ROTATE_REASON_USER_COMMAND, lastRecError, lastTecError);
    sendToUILn("Manual diagnostic file created");
  }
}

void sendStatus() {
  unsigned long now = millis();
  unsigned long runtime = (now - startTime) / 1000;
  
  // Check if time is synchronized
  time_t now_time = time(nullptr);
  struct tm *timeinfo = localtime(&now_time);
  bool timeSynced = (timeinfo->tm_year >= 120);  // year >= 2020
  
  sendToUILn("STATUS_BEGIN");
  delay(5);
  sendToUI("Uptime: "); sendToUI(String(runtime).c_str()); sendToUILn(" s");
  sendToUI("Free heap: "); sendToUILn(String(ESP.getFreeHeap()).c_str());
  sendToUI("SD Card: "); sendToUILn(sdReady ? "Ready" : "Not Found");
  sendToUI("Time Sync: "); sendToUILn(timeSynced ? "OK" : "Waiting for GPS");
  sendToUI("Logging: "); sendToUILn(loggingActive ? "Active" : "Stopped");
  sendToUI("Messages: "); sendToUILn(String(messageCount).c_str());
  sendToUI("Accepted: "); sendToUILn(String(acceptedCount).c_str());
  sendToUI("Filtered: "); sendToUILn(String(filteredOutCount).c_str());
  sendToUI("Logged: "); sendToUILn(String(loggedCount).c_str());
  sendToUI("ECU State: "); sendToUILn(String(ecuState).c_str());
  sendToUI("Session ID: "); sendToUILn(String(currentSessionId).c_str());
  sendToUI("File Sequence: "); sendToUILn(String(currentFileSequence).c_str());
  sendToUI("Session Records: "); sendToUILn(String(sessionRecordCounter).c_str());
  sendToUI("File Records: "); sendToUILn(String(fileRecordCounter).c_str());
  sendToUILn("STATUS_END");
}

void sendCardInfo() {
  if (!sdReady) {
    sendToUILn("ERROR: SD not ready");
    return;
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  uint64_t totalSize = SD.totalBytes() / (1024 * 1024);
  uint64_t usedSize = SD.usedBytes() / (1024 * 1024);
  
  sendToUILn("SD_CARD_INFO_BEGIN");
  sendToUI("Card size: "); sendToUI(String(cardSize).c_str()); sendToUILn(" MB");
  sendToUI("Total: "); sendToUI(String(totalSize).c_str()); sendToUILn(" MB");
  sendToUI("Used: "); sendToUI(String(usedSize).c_str()); sendToUILn(" MB");
  sendToUI("Free: "); sendToUI(String(totalSize - usedSize).c_str()); sendToUILn(" MB");
  sendToUILn("SD_CARD_INFO_END");
}

void sendHelp() {
  sendToUILn("=== BMS LOGGER COMMANDS ===");
  delay(5);
  sendToUILn("list           - List SD files");
  delay(5);
  sendToUILn("read [file]    - Read file");
  delay(5);
  sendToUILn("delete [file]  - Delete file");
  delay(5);
  sendToUILn("create [file]  - Create test file");
  delay(5);
  sendToUILn("info           - SD card info");
  delay(5);
  sendToUILn("status         - System status");
  delay(5);
  sendToUILn("live           - Live data");
  delay(5);
  sendToUILn("stats          - Statistics");
  delay(5);
  sendToUILn("logstart       - Start logging");
  delay(5);
  sendToUILn("logstop        - Stop logging");
  delay(5);
  sendToUILn("reset          - Reset stats");
  delay(5);
  sendToUILn("logstatus      - Show current log file info");
  delay(5);
  sendToUILn("logrotate      - Force rotate current log");
  delay(5);
  sendToUILn("loglist YYYY-MM-DD - List logs by date");
  delay(5);
  sendToUILn("sessions       - Show session history");  // NEW
  delay(5);
  sendToUILn("listdiag       - List all diagnostic files");
  delay(5);
  sendToUILn("creatediag     - Create a manual diagnostic file");
  delay(5);
  sendToUILn("logsummary     - Show session history");
  delay(5);
  sendToUILn("help           - This help");
}

void sendLiveData() {
  unsigned long now = millis();
  
  sendToUILn("LIVE_DATA_BEGIN");
  delay(5);
  
  if (isValid(battSt1.lastUpdate, battSt1.timeoutMs, now)) {
    sendToUI("BATT:"); 
    sendToUI(String(battSt1.voltage, 2).c_str()); sendToUI(",");
    sendToUI(String(battSt1.current, 2).c_str()); sendToUI(",");
    sendToUILn(String(battSt1.soc).c_str());
  }
  
  if (isValid(cellVolt.lastUpdate, cellVolt.timeoutMs, now)) {
    sendToUI("CELL_VOLT:");
    sendToUI(String(cellVolt.maxCellNo).c_str()); sendToUI(",");
    sendToUI(String(cellVolt.maxCellVolt).c_str()); sendToUI(",");
    sendToUI(String(cellVolt.minCellNo).c_str()); sendToUI(",");
    sendToUILn(String(cellVolt.minCellVolt).c_str());
  }
  
  if (isValid(mcuMsg1.lastUpdate, mcuMsg1.timeoutMs, now)) {
    sendToUI("MCU:");
    sendToUI(String(mcuMsg1.dcVolt).c_str()); sendToUI(",");
    sendToUI(String(mcuMsg1.motorTemp).c_str()); sendToUI(",");
    sendToUI(String(mcuMsg1.cntrlTemp).c_str()); sendToUI(",");
    sendToUILn(String(mcuMsg1.throttlePercent).c_str());
  }
  
  if (isValid(mcuMsg2.lastUpdate, mcuMsg2.timeoutMs, now)) {
    sendToUI("SPEED:");
    sendToUI(String(mcuMsg2.motorSpeed).c_str()); sendToUI(",");
    sendToUILn(String(mcuMsg2.motorSpdLim).c_str());
  }
  
  sendToUI("ECU_STATE:"); sendToUILn(String(ecuState).c_str());
  
  sendToUI("SESSION_INFO:");
  sendToUI(String(currentSessionId).c_str()); sendToUI(",");
  sendToUI(String(currentFileSequence).c_str()); sendToUI(",");
  sendToUI(String(sessionRecordCounter).c_str()); sendToUI(",");
  sendToUILn(String(fileRecordCounter).c_str());
  
  sendToUILn("LIVE_DATA_END");
}

void sendStats() {
  unsigned long runtime = (millis() - startTime) / 1000;
  
  Serial2.println("STATS_BEGIN");
  delay(5);
  Serial2.print("Runtime:"); Serial2.println(runtime);
  Serial2.print("Messages:"); Serial2.println(messageCount);
  Serial2.print("Accepted:"); Serial2.println(acceptedCount);
  Serial2.print("Filtered:"); Serial2.println(filteredOutCount);
  Serial2.print("Logged:"); Serial2.println(loggedCount);
  Serial2.print("SD_Ready:"); Serial2.println(sdReady ? "1" : "0");
  Serial2.print("Logging:"); Serial2.println(loggingActive ? "1" : "0");
  Serial2.print("ECU_State:"); Serial2.println(ecuState);
  Serial2.print("SessionID:"); Serial2.println(currentSessionId);
  Serial2.print("FileSeq:"); Serial2.println(currentFileSequence);
  Serial2.print("SessionRec:"); Serial2.println(sessionRecordCounter);
  Serial2.print("FileRec:"); Serial2.println(fileRecordCounter);
  
  if (sdReady) {
    File root = SD.open("/");
    if (root) {
      int fileCount = 0;
      File file = root.openNextFile();
      while (file) {
        if (!file.isDirectory()) fileCount++;
        file.close();
        file = root.openNextFile();
      }
      root.close();
      Serial2.print("Files:"); Serial2.println(fileCount);
    }
  }
  
  Serial2.println("STATS_END");
}

void sendGPSInfo() {
    sendToUILn("GPS_INFO_BEGIN");
    sendToUI("Initialized: "); sendToUILn(gpsInitialized ? "Yes" : "No");
    sendToUI("Location: "); 
    if (gpsData.location_valid) {
        char locStr[64];
        snprintf(locStr, sizeof(locStr), "%.6f, %.6f", gpsData.latitude, gpsData.longitude);
        sendToUILn(locStr);
    } else {
        sendToUILn("Invalid");
    }
    sendToUI("Time (UTC): ");
    if (gpsData.time_valid) {
        char timeStr[20];
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", 
                 gpsData.hour_utc, gpsData.minute_utc, gpsData.second_utc);
        sendToUILn(timeStr);
    } else {
        sendToUILn("Invalid");
    }
    sendToUI("Speed: ");
    if (gpsData.speed_valid) {
        char speedStr[20];
        snprintf(speedStr, sizeof(speedStr), "%.1f km/h", gpsData.speed_kmh);
        sendToUILn(speedStr);
    } else {
        sendToUILn("Invalid");
    }
    sendToUI("Satellites: ");
    if (gpsData.satellites_valid) {
        sendToUILn(String(gpsData.satellites).c_str());
    } else {
        sendToUILn("Invalid");
    }
    sendToUI("HDOP: ");
    if (gpsData.hdop_valid) {
        sendToUILn(String(gpsData.hdop, 1).c_str());
    } else {
        sendToUILn("Invalid");
    }
    sendToUILn("GPS_INFO_END");
}

void applyConfigurationChanges() {
    // Apply logging interval change
    // Note: LOG_INTERVAL_MS is a #define, so we need to use the runtime variable
    // The logging task already uses logIntervalMs, so no change needed
    
    // Apply GPS baud rate (would require restarting GPS)
    // For now, just print a message
    if (gpsInitialized) {
        Serial.printf("GPS baud rate changed to %d - will apply on next GPS restart\n", gpsBaudRate);
        // You could restart the GPS task here, but that's complex
        // For simplicity, just note that it requires restart
    }
    
    // Apply GPS update interval - this is used in the GPS task
    // The GPS task reads gpsUpdateInterval, so it will take effect immediately
    
    // Apply buffer size - this would require reallocating buffers
    // For now, just note that it requires restart
    Serial.printf("Buffer size changed to %d - will apply on next restart\n", bufferSize);
    
    // Apply ECU timeout - used in ECU state machine
    // The ECU state machine reads ecuTimeout, so it will take effect immediately
    
    // Apply WiFi settings - would require reconnecting
    if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
        Serial.printf("WiFi settings changed - would need to reconnect\n");
        // You could trigger WiFi reconnection here
    }
    
    // Update file rotation settings
    // These are used in needsFileRotation(), so they take effect immediately
    
    sendToUILn("CONFIG_APPLIED");
}

void sendSessionHistory() {
    if (!sdReady) {
        sendToUILn("ERROR: SD not ready");
        return;
    }
    
    if (!SD.exists(sessionLogPath)) {
        sendToUILn("SESSION_HISTORY_BEGIN");
        sendToUILn("NO_SESSIONS");
        sendToUILn("SESSION_HISTORY_END");
        return;
    }
    
    File file = SD.open(sessionLogPath, FILE_READ);
    if (!file) {
        sendToUILn("ERROR: Cannot open session log");
        return;
    }
    
    sendToUILn("SESSION_HISTORY_BEGIN");
    
    // Read and send all session records
    bool firstLine = true;
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            // Skip the header line if present
            if (firstLine && line.startsWith("SessionID")) {
                firstLine = false;
                continue;
            }
            firstLine = false;
            sendToUILn(line.c_str());
        }
    }
    
    file.close();
    sendToUILn("SESSION_HISTORY_END");
}