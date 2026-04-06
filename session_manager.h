#ifndef SESSION_MANAGER_H
#define SESSION_MANAGER_H

#include <Arduino.h>
#include "types.h"

// ================ SESSION GLOBALS ================
extern uint32_t currentSessionId;
extern uint32_t currentFileSequence;
extern uint32_t sessionRecordCounter;
extern uint32_t fileRecordCounter;
extern SessionState_t sessionState;
extern SessionMetadata_t currentSession;
extern bool firstConnection;
extern bool recoveryMode;

// ================ FUNCTION PROTOTYPES ================
void initSessionManager();
void startNewSession();
void closeCurrentSession(RotateReason_t reason);
void incrementSessionRecords();
void incrementFileRecords();
void handleECUConnection();
void handleECUDisconnection();
void updateSessionMetadata();
void printSessionStatus();

#endif