#ifndef CONFIG_H
#define CONFIG_H

// ================ BUFFER CONFIG ================
#define BUFFER_SIZE 16384   // 16 KB per buffer
#define FLUSH_INTERVAL 2000    // Flush every 2 seconds
// Debug output control: set to 1 to enable verbose debug prints, 0 to disable
#define ENABLE_SERIAL_DEBUG 0

// ================ FILE MANAGEMENT CONFIG ================
#define MAX_FILE_SIZE_MB 100
#define MAX_FILE_SIZE (MAX_FILE_SIZE_MB * 1024 * 1024)
#define ROTATE_HOURLY false
#define INCLUDE_DATE_IN_FILENAME true
#define MAX_LOG_FILES 1000
#define AUTO_DELETE_OLD_DAYS 30
#define ECU_DISCONNECT_TIMEOUT 30000
#define ECU_RECONNECT_GRACE 10000

// In config.h - add this line
#define CSV_LINE_BUFFER_SIZE 2048   // Increased buffer size for I2C sensors

// ================ DATA LOGGING CONFIG ================
#define LOG_INTERVAL_MS 100
#define PRINT_EVERY_MESSAGE false
#define PRINT_LOGGED_DATA false

// ================ CAN FILTER CONFIG ================
#define FILTER_MODE 1

// ================ DBC CONFIG ================
#define MAX_DBC_MESSAGES 100
#define MAX_DBC_SIGNALS_PER_MESSAGE 50
#define MAX_SELECTED_SIGNALS 200
#define DBC_UPLOAD_PATH "/dbc/upload.dbc"
#define SIGNAL_CONFIG_PATH "/config/signals.json"

// Add after existing DBC defines
#define UART_UPLOAD_PATH "/uart/upload.csv"
#define UART_MESSAGES_PATH "/uart/messages.json"
#define UART_SIGNAL_CONFIG_PATH "/config/uart_signals.json"
#define UART_ID_BASE 0x80000000   // Reserve a range for UART messages

// I2C Sensor Configuration
#define I2C_UPDATE_INTERVAL_MS 100  // Update I2C sensors every 100 milli second

#endif