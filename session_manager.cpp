#include "session_manager.h"
#include "file_manager.h"
#include "ecu_state.h"
#include "utils.h"
#include "globals.h"
#include <time.h>   

// ================ SESSION GLOBALS ================
uint32_t currentSessionId = 0;
uint32_t currentFileSequence = 0;
uint32_t sessionRecordCounter = 0;
uint32_t fileRecordCounter = 0;
SessionState_t sessionState = SESSION_STATE_BOOT;
SessionMetadata_t currentSession;
bool firstConnection = true;
bool recoveryMode = false;

extern ECUState_t ecuState;

void initSessionManager() {
  currentSessionId = 0;
  currentFileSequence = 0;
  sessionRecordCounter = 0;
  fileRecordCounter = 0;
  sessionState = SESSION_STATE_WAITING;
  firstConnection = true;
  recoveryMode = false;
  
  memset(&currentSession, 0, sizeof(SessionMetadata_t));
}

void startNewSession() {
  currentSessionId++;
  currentFileSequence = 0;
  sessionRecordCounter = 0;
  fileRecordCounter = 0;
  
  currentSession.sessionId = currentSessionId;
  currentSession.startEpoch = time(nullptr);   
  currentSession.ecuState = ecuState;
  
  Serial.printf("New session %u started\n", currentSessionId);
}

void closeCurrentSession(RotateReason_t reason) {
  currentSession.endEpoch = time(nullptr);     
  currentSession.sessionRecordCount = sessionRecordCounter;
  currentSession.ecuState = ecuState;
  currentSession.rotateReason = reason;
  currentSession.cleanClosure = true;
  
  Serial.printf("Session %u closed: %u records\n", 
                currentSession.sessionId, sessionRecordCounter);
}

void incrementSessionRecords() {
  sessionRecordCounter++;
}

void incrementFileRecords() {
  fileRecordCounter++;
}

void handleECUConnection() {
  
  Serial.println("ECU connected (waiting for filtered signals)");
}

void handleECUDisconnection() {
 
}

void updateSessionMetadata() {
  currentSession.fileSequence = currentFileSequence;
  currentSession.fileRecordCount = fileRecordCounter;
  currentSession.fileSize = 0; 
}

void printSessionStatus() {
  Serial.printf("Session %u, File %u, Records: Session=%u File=%u\n",
                currentSessionId, currentFileSequence,
                sessionRecordCounter, fileRecordCounter);
}