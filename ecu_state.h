#ifndef ECU_STATE_H
#define ECU_STATE_H

#include <Arduino.h>
#include "types.h"
#include "driver/twai.h"

// ================ ECU STATE GLOBALS ================
extern ECUState_t ecuState;
extern unsigned long ecuDisconnectTimer;
extern unsigned long ecuLastMessageTime;
extern int ecuDisconnectSampleCount;
extern int ecuConnectSampleCount;
extern uint32_t lastRecError;
extern uint32_t lastTecError;
extern uint32_t recErrorAccumulator;
extern uint32_t tecErrorAccumulator;
extern uint32_t errorSampleCount;
extern bool busOffDetected;

// ================ FUNCTION PROTOTYPES ================
void initECUState();
void checkECUState();
void updateECUState(bool messageReceived, uint32_t avgRecError, uint32_t avgTecError);
const char* getECUStateString(ECUState_t state);

#endif