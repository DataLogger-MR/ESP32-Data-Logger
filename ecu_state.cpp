#include "ecu_state.h"
#include "session_manager.h"
#include "file_manager.h"
#include "config.h"
#include "globals.h"

// ================ ECU STATE GLOBALS ================
ECUState_t ecuState = ECU_STATE_UNKNOWN;
unsigned long ecuDisconnectTimer = 0;
unsigned long ecuLastMessageTime = 0;   // now based on filteredMessageCount
int ecuDisconnectSampleCount = 0;
int ecuConnectSampleCount = 0;
uint32_t lastRecError = 0;
uint32_t lastTecError = 0;
uint32_t recErrorAccumulator = 0;
uint32_t tecErrorAccumulator = 0;
uint32_t errorSampleCount = 0;
bool busOffDetected = false;

// External references
extern unsigned long filteredMessageCount;   // <-- use filtered count
extern SessionState_t sessionState;
extern void rotateFile(RotateReason_t reason);

void initECUState() {
  ecuState = ECU_STATE_UNKNOWN;
  ecuDisconnectTimer = 0;
  ecuLastMessageTime = 0;
  ecuDisconnectSampleCount = 0;
  ecuConnectSampleCount = 0;
  lastRecError = 0;
  lastTecError = 0;
  recErrorAccumulator = 0;
  tecErrorAccumulator = 0;
  errorSampleCount = 0;
  busOffDetected = false;
}

void checkECUState() {
  static unsigned long lastECUCheck = 0;
  unsigned long now = millis();
  
  if (now - lastECUCheck < 100) return;
  lastECUCheck = now;
  
  twai_status_info_t twai_status;
  if (twai_get_status_info(&twai_status) == ESP_OK) {
    lastRecError = twai_status.rx_error_counter;
    lastTecError = twai_status.tx_error_counter;
    busOffDetected = (twai_status.state == TWAI_STATE_BUS_OFF);
    
    recErrorAccumulator += lastRecError;
    tecErrorAccumulator += lastTecError;
    errorSampleCount++;
  }
  
  // Use filteredMessageCount instead of messageCount
  bool messageReceived = (filteredMessageCount > ecuLastMessageTime);
  if (messageReceived) {
    ecuLastMessageTime = filteredMessageCount;
    ecuDisconnectSampleCount = 0;
    ecuConnectSampleCount++;
  } else {
    ecuConnectSampleCount = 0;
    ecuDisconnectSampleCount++;
  }
  
  uint32_t avgRecError = 0;
  uint32_t avgTecError = 0;
  if (errorSampleCount >= 10) {
    avgRecError = recErrorAccumulator / errorSampleCount;
    avgTecError = tecErrorAccumulator / errorSampleCount;
    recErrorAccumulator = 0;
    tecErrorAccumulator = 0;
    errorSampleCount = 0;
  }
  
  updateECUState(messageReceived, avgRecError, avgTecError);

  // ===== ADDED: Check for disconnect timeout while in DISCONNECTED state =====
      if (ecuState == ECU_STATE_DISCONNECTED) {
          if (ecuDisconnectTimer != 0 && (now - ecuDisconnectTimer) > ecuTimeout) {
              Serial.printf("%d ms disconnect timeout triggered, rotating file\n", ecuTimeout);
              rotateFile(ROTATE_REASON_ECU_DISCONNECT);
              ecuDisconnectTimer = 0;
          }
  }
  // ===========================================================================
}

void updateECUState(bool messageReceived, uint32_t avgRecError, uint32_t avgTecError) {
  ECUState_t newState = ecuState;
  unsigned long now = millis();
  
  if (busOffDetected) {
    newState = ECU_STATE_DISCONNECTED;
    ecuDisconnectTimer = now;
  }
  else if (messageReceived) {
    if (avgRecError < 10 && avgTecError < 10) {
      newState = ECU_STATE_CONNECTED;
    } else if (avgRecError > 50 || avgTecError > 50) {
      newState = ECU_STATE_DEGRADED;
    } else {
      newState = ECU_STATE_CONNECTED;
    }
    ecuDisconnectTimer = 0;
  }
  else {
    if (ecuDisconnectTimer == 0) {
      ecuDisconnectTimer = now;
    }
    
    if (avgRecError > 100 || avgTecError > 100) {
      newState = ECU_STATE_DISCONNECTED;
    }
    else if (avgRecError < 10 && avgTecError < 10) {
      if (ecuDisconnectSampleCount > 50) {
        newState = ECU_STATE_SILENT;
      }
    }
    else {
      newState = ECU_STATE_UNKNOWN;
    }
  }
  
  if (newState != ecuState) {
    if (newState == ECU_STATE_CONNECTED) {
      if (ecuConnectSampleCount >= 3) {
        ecuState = newState;
        Serial.printf("ECU State: CONNECTED (errors: %d/%d)\n", avgRecError, avgTecError);
        handleECUConnection();
      }
    } else if (newState == ECU_STATE_DISCONNECTED) {
      if (ecuDisconnectSampleCount >= 3) {
        ecuState = newState;
        Serial.printf("ECU State: DISCONNECTED (errors: %d/%d)\n", avgRecError, avgTecError);
      }
    } else {
      ecuState = newState;
    }
  }
}

const char* getECUStateString(ECUState_t state) {
  switch(state) {
    case ECU_STATE_UNKNOWN: return "Unknown";
    case ECU_STATE_CONNECTED: return "Connected";
    case ECU_STATE_DISCONNECTED: return "Disconnected";
    case ECU_STATE_DEGRADED: return "Degraded";
    case ECU_STATE_SILENT: return "Silent";
    default: return "Unknown";
  }
}