#ifndef PINS_H
#define PINS_H

#define SENSOR_UART_RX 16   // Choose free GPIO (e.g., 14)
#define SENSOR_UART_TX 17   // Choose free GPIO (e.g., 15)


#define Encoder_PWM 12  // Hall sensor input
// ================ PIN CONFIG ================
#define CAN_RX_PIN 4
#define CAN_TX_PIN 32
#define SD_CS 5
#define LED_PIN 2

// USB TTL Pins for UI Communication
#define UI_RXD2 14  // GPIO16 - Connect to USB-TTL TX (UI Receive)
#define UI_TXD2 15  // GPIO17 - Connect to USB-TTL RX (UI Transmit)
// SD Card SPI Pins
#define SD_SCK 18
#define SD_MOSI 23
#define SD_MISO 19

#endif