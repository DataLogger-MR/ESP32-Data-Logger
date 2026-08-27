// torque_rs485.h
#ifndef TORQUE_RS485_H
#define TORQUE_RS485_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// Frame from aux (14 bytes, little-endian)
#define TORQUE_FRAME_LEN    14
#define TORQUE_SYNC0        0xAA
#define TORQUE_SYNC1        0x55
#define TORQUE_PAYLOAD_LEN  12   // status..age..crc (11 data bytes + 1 crc)

// Status values
#define TORQUE_STATUS_OK        0
#define TORQUE_STATUS_STALE     1
#define TORQUE_STATUS_NO_DATA   2

typedef struct {
    float torqueNm;          // latest valid torque in Nm
    int32_t torque_mNm;      // raw torque in milli-Nm
    uint32_t sampleCounter;  // sample counter from aux
    uint8_t status;          // 0=OK, 1=STALE, 2=NO_DATA
    uint16_t ageMs;          // age in ms since last good read
    bool valid;              // true if we have a valid frame
    bool fresh;              // set after parse, cleared by consumer
    uint32_t goodFrames;     // number of good frames received
    uint32_t crcErrors;      // number of CRC errors
    uint32_t frameRate;      // frames per second (updated every 1s)
} TorqueData_t;

// g_torque is defined in globals.cpp, declared as extern in globals.h
// This is just a forward declaration for use by other files
extern TorqueData_t g_torque;

esp_err_t torque_begin(void);
void torque_service(void);
void torque_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif