#ifndef PINS_H
#define PINS_H

// ================ UI - Serial1 on GPIO14, GPIO15 ================
#define UI_RX_PIN 14
#define UI_TX_PIN 15

// ================ GPS - SoftwareSerial on GPIO16, GPIO17 ================
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// ================ CAN BUS ================
#define CAN_RX_PIN 4
#define CAN_TX_PIN 32

// ================ SD CARD ================
#define SD_CS 5
#define SD_SCK 18
#define SD_MOSI 23
#define SD_MISO 19

// ================ LED ================
#define LED_PIN 2

// ================ SPEED SENSORS ================
#define Encoder_PWM 13
#define AUX_Encoder_PWM 33

// ================ TORQUE SENSOR (UART2) ================
#define TORQUE_RX_PIN 25
#define TORQUE_TX_PIN 26

#endif