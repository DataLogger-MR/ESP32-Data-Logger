// torque_rs485.cpp
#include "torque_rs485.h"
#include "esp_err.h"
#include <Arduino.h>

// ---------- Pins ----------
#define LINK_RX   25
#define LINK_TX   26
#define LINK_BAUD 115200

// ---------- Parser state ----------
enum { WAIT_SYNC0, WAIT_SYNC1, READ_PAYLOAD };

static uint8_t state = WAIT_SYNC0;
static uint8_t buf[TORQUE_PAYLOAD_LEN];
static uint8_t idx = 0;
static uint32_t framesThisSec = 0;
static uint32_t lastRateMs = 0;

// ---------- Global Torque Data ----------
// g_torque is defined in globals.cpp, we just reference it here

// ---------- CRC8 ----------
static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// ---------- Handle a complete frame ----------
static void handleFrame(void) {
    // buf[0..10] = payload, buf[11] = received crc
    uint8_t calc = crc8(buf, 11);
    if (calc != buf[11]) {
        g_torque.crcErrors++;
        g_torque.valid = false;
        return;
    }

    uint8_t  status  = buf[0];
    uint32_t counter = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8) |
                       ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
    int32_t  torque  = (int32_t)((uint32_t)buf[5] | ((uint32_t)buf[6] << 8) |
                       ((uint32_t)buf[7] << 16) | ((uint32_t)buf[8] << 24));
    uint16_t age     = (uint16_t)buf[9] | ((uint16_t)buf[10] << 8);

    // Update global torque data
    g_torque.status = status;
    g_torque.sampleCounter = counter;
    g_torque.torque_mNm = torque;
    g_torque.torqueNm = torque / 1000.0f;
    g_torque.ageMs = age;
    g_torque.valid = true;
    g_torque.fresh = true;
    g_torque.goodFrames++;
    framesThisSec++;

    // ===== DEBUG PRINT - Shows each valid frame =====
   // Serial.printf("[Torque] status=%d torque=%.1f Nm cnt=%lu age=%u ms\n",
       //           status, g_torque.torqueNm, (unsigned long)counter, age);
}

// ---------- Public API ----------
esp_err_t torque_begin(void) {
    // Serial2 is used by the torque sensor on pins 25 (RX) and 26 (TX)
    // The UI uses SoftwareSerial on pins 14,15, so no conflict.
    // We DO NOT use GPIO27 anymore (DE/RE is removed).
    Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
    Serial2.setTimeout(10);

    state = WAIT_SYNC0;
    idx = 0;
    lastRateMs = millis();
    framesThisSec = 0;

    // Clear torque data (g_torque is already zero-initialized in globals.cpp)
    g_torque.goodFrames = 0;
    g_torque.crcErrors = 0;
    g_torque.frameRate = 0;
    g_torque.valid = false;
    g_torque.fresh = false;

    Serial.println("✅ Torque sensor initialized (UART2, 115200 baud, read-only)");
    return ESP_OK;
}

void torque_service(void) {
    uint32_t now = millis();

    // Process all available bytes
    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        switch (state) {
            case WAIT_SYNC0:
                if (b == TORQUE_SYNC0) state = WAIT_SYNC1;
                break;

            case WAIT_SYNC1:
                if (b == TORQUE_SYNC1) {
                    state = READ_PAYLOAD;
                    idx = 0;
                } else if (b == TORQUE_SYNC0) {
                    state = WAIT_SYNC1;   // possible new start
                } else {
                    state = WAIT_SYNC0;
                }
                break;

            case READ_PAYLOAD:
                buf[idx++] = b;
                if (idx >= TORQUE_PAYLOAD_LEN) {
                    handleFrame();
                    state = WAIT_SYNC0;
                }
                break;
        }
    }

    // Once per second, report reception rate
    if (now - lastRateMs >= 1000) {
        lastRateMs = now;
        g_torque.frameRate = framesThisSec;
        framesThisSec = 0;

        // Optional: keep the rate debug or comment it out
       //  Serial.printf("[Torque] rate=%lu fps  good=%lu  crcErr=%lu\n",
        //               (unsigned long)g_torque.frameRate,
          //             (unsigned long)g_torque.goodFrames,
            //           (unsigned long)g_torque.crcErrors);
    }
}

void torque_reset_stats(void) {
    g_torque.goodFrames = 0;
    g_torque.crcErrors = 0;
    g_torque.frameRate = 0;
}