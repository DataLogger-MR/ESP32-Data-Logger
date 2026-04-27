#include "can_decoder.h"
#include "utils.h"
#include "config.h"
#include "pins.h"
#include "globals.h"
#include "dynamic_decoder.h"
#include "session_manager.h"
#include "file_manager.h"
#include <map>
#include <Arduino.h>

extern std::map<uint32_t, std::vector<DBCSignal>> activeSignals;
extern HardwareSerial Serial2;

extern std::map<String, double> lastDynamicValues;

twai_status_info_t twai_status;

// ================ CAN FILTER DEFINITIONS ================
const CanFilter_t FILTERED_IDS[] = {
  
  {0x2F4, false, "BATT_ST1", 100},
  {0x4F4, false, "CELL_VOLT", 500},
  {0x5F4, false, "CELL_TEMP", 2500},
  {0x7F4, false, "ALM_INFO", 500},
  
  {0x08F4, true, "BMS_6", 500},
  {0x18F128F4, true, "BATT_ST2", 500},
  {0x18F228F4, true, "ALL_TEMP", 2500},
  {0x18F328F4, true, "BMSERR_INFO", 500},
  {0x18F428F4, true, "BMS_INFO", 2500},
  {0x18F528F4, true, "BmsSwSta", 2500},
  {0x18E028F4, true, "CellVol_1", 5000},
  {0x18E128F4, true, "CellVol_2", 5000},
  {0x18E228F4, true, "CellVol_3", 5000},
  {0x18E328F4, true, "CellVol_4", 5000},
  {0x18E428F4, true, "CellVol_5", 5000},
  {0x18E528F4, true, "CellVol_6", 5000},
  {0x18E628F4, true, "CellVol_7", 5000},
  {0x1806E5F4, true, "BMSChgINFO", 2500},
  {0x18F0F428, true, "Ctrl_INFO", 2500},
  
  {0x102200A0, true, "MCU_MSG_1", 250},
  {0x102200A1, true, "MCU_MSG_2", 250},
  {0x102200A2, true, "MCU_MSG_3", 5000},
  {0x102200A3, true, "MCU_MSG_4", 250},
  {0x102200A4, true, "MCU_MSG_5", 500},
  {0x102200A5, true, "MCU_MSG_6", 500},
  
  {0x19FF50F0, true, "AUX_MOTOR_1", 50},
  {0x19FF50F1, true, "AUX_MOTOR_2", 50},
  {0x19FF50F2, true, "AUX_MOTOR_3", 50},
  
  {0x18FF50E5, true, "CHRG_OUT", 2500},
};

const int FILTER_COUNT = sizeof(FILTERED_IDS) / sizeof(FILTERED_IDS[0]);

// ================ GLOBAL DATA INSTANCES ================
BattSt1_t battSt1 = {0, 0, 0, false, 0, 100};
CellVolt_t cellVolt = {0, 0, 0, 0, false, 0, 500};
CellTemp_t cellTemp = {0, 0, 0, 0, 0, false, 0, 2500};
AlmInfo_t almInfo = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0, 500};
Bms6_t bms6 = {0, 0, 0, 0, false, 0, 500};
BattSt2_t battSt2 = {0, 0, 0, 0, false, 0, 500};
AllTemp_t allTemp = {0, 0, 0, 0, 0, 0, false, 0, 2500};
BmsErrInfo_t bmsErrInfo = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, 0, 0, false, 0, 500};
BmsInfo_t bmsInfo = {0, 0, 0, false, 0, 2500};
BmsSwSta_t bmsSwSta = {false, false, false, false, false, false, false, 0, 2500};
CellVoltages_t cellVoltages = {{{0}}, 0, false, 0, 5000};
BmsChgInfo_t bmsChgInfo = {0, 0, false, false, false, 0, 2500};
CtrlInfo_t ctrlInfo = {0, false, false, false, false, 0, 2500};
McuMsg1_t mcuMsg1 = {0, 0, 0, 0, false, 0, 250};
McuMsg2_t mcuMsg2 = {0, 0, 0, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, 0, 250};
McuMsg3_t mcuMsg3 = {0, 0, 0, false, 0, 5000};
McuMsg4_t mcuMsg4 = {0, 0, false, false, false, false, false, false, false, false, false, false, 0, 250};
McuMsg5_t mcuMsg5 = {0, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, 0, 500};
McuMsg6_t mcuMsg6 = {0, false, 0, 500};
AuxMotor1_t auxMotor1 = {0, 0, 0, 0, false, 0, 50};
AuxMotor2_t auxMotor2 = {0, 0, 0, 0, false, 0, 50};
AuxMotor3_t auxMotor3 = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, 0, 0, false, 0, 50};
ChrgOut_t chrgOut = {0, 0, false, false, false, false, false, false, false, false, 0, 0, false, 0, 2500};

// ================ CAN FILTER FUNCTIONS ================
bool acceptCANId(uint32_t id, bool isExtended) {
  #if FILTER_MODE == 0
    return true;
  #else
    for (int i = 0; i < FILTER_COUNT; i++) {
      if (id == FILTERED_IDS[i].id && isExtended == FILTERED_IDS[i].isExtended) {
        return true;
      }
    }
    return false;
  #endif
}

const char* getMessageName(uint32_t id, bool isExtended) {
  for (int i = 0; i < FILTER_COUNT; i++) {
    if (id == FILTERED_IDS[i].id && isExtended == FILTERED_IDS[i].isExtended) {
      return FILTERED_IDS[i].name;
    }
  }
  return "UNKNOWN";
}

void initCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, 
    (gpio_num_t)CAN_RX_PIN, 
    TWAI_MODE_NORMAL
  );
  g_config.rx_queue_len = canRxQueueSize;
  
  twai_timing_config_t t_config;
  
  switch(canBaudRate) {
    case 125:
      t_config = TWAI_TIMING_CONFIG_125KBITS();
      break;
    case 250:
      t_config = TWAI_TIMING_CONFIG_250KBITS();
      break;
    case 500:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;
    case 1000:
      t_config = TWAI_TIMING_CONFIG_1MBITS();
      break;
    default:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      Serial.printf("Unknown CAN baud %d, using 500kbps\n", canBaudRate);
  }
  
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial2.println("CAN_OK");
  } else {
    Serial.println("❌ CAN initialization FAILED!");
    Serial2.println("CAN_ERROR");
  }
}

void initializeTimeouts() {
  for (int i = 0; i < FILTER_COUNT; i++) {
    uint32_t id = FILTERED_IDS[i].id;
    bool isExtended = FILTERED_IDS[i].isExtended;
    uint16_t timeout = FILTERED_IDS[i].timeoutMs;
    
    if (!isExtended) {
      switch (id) {
        case 0x2F4: battSt1.timeoutMs = timeout; break;
        case 0x4F4: cellVolt.timeoutMs = timeout; break;
        case 0x5F4: cellTemp.timeoutMs = timeout; break;
        case 0x7F4: almInfo.timeoutMs = timeout; break;
      }
    } else {
      switch (id) {
        case 0x08F4: bms6.timeoutMs = timeout; break;
        case 0x18F128F4: battSt2.timeoutMs = timeout; break;
        case 0x18F228F4: allTemp.timeoutMs = timeout; break;
        case 0x18F328F4: bmsErrInfo.timeoutMs = timeout; break;
        case 0x18F428F4: bmsInfo.timeoutMs = timeout; break;
        case 0x18F528F4: bmsSwSta.timeoutMs = timeout; break;
        case 0x18E028F4: 
        case 0x18E128F4:
        case 0x18E228F4:
        case 0x18E328F4:
        case 0x18E428F4:
        case 0x18E528F4:
        case 0x18E628F4: cellVoltages.timeoutMs = timeout; break;
        case 0x1806E5F4: bmsChgInfo.timeoutMs = timeout; break;
        case 0x18F0F428: ctrlInfo.timeoutMs = timeout; break;
        case 0x102200A0: mcuMsg1.timeoutMs = timeout; break;
        case 0x102200A1: mcuMsg2.timeoutMs = timeout; break;
        case 0x102200A2: mcuMsg3.timeoutMs = timeout; break;
        case 0x102200A3: mcuMsg4.timeoutMs = timeout; break;
        case 0x102200A4: mcuMsg5.timeoutMs = timeout; break;
        case 0x102200A5: mcuMsg6.timeoutMs = timeout; break;
        case 0x19FF50F0: auxMotor1.timeoutMs = timeout; break;
        case 0x19FF50F1: auxMotor2.timeoutMs = timeout; break;
        case 0x19FF50F2: auxMotor3.timeoutMs = timeout; break;
        case 0x18FF50E5: chrgOut.timeoutMs = timeout; break;
      }
    }
  }
}

// ================ DECODER FUNCTIONS ================

void decodeBattSt1(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  uint16_t rawVoltage = getIntel16(msg.data, 0);
  uint16_t rawCurrent = getIntel16(msg.data, 2);
  uint8_t rawSoc = msg.data[4];
  
  battSt1.voltage = rawVoltage * 0.1;
  battSt1.current = (rawCurrent * 0.1 - 400);
  battSt1.soc = rawSoc;
  battSt1.valid = true;
  battSt1.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeCellVolt(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  cellVolt.maxCellVolt = getIntel16(msg.data, 0);
  cellVolt.maxCellNo = msg.data[2];
  cellVolt.minCellVolt = getIntel16(msg.data, 3);
  cellVolt.minCellNo = msg.data[5];
  cellVolt.valid = true;
  cellVolt.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeCellTemp(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  cellTemp.maxCellTemp = (int8_t)msg.data[0] - 50;
  cellTemp.maxCtNO = msg.data[1];
  cellTemp.minCellTemp = (int8_t)msg.data[2] - 50;
  cellTemp.minCtNO = msg.data[3];
  cellTemp.avgCellTemp = (int8_t)msg.data[4] - 50;
  cellTemp.valid = true;
  cellTemp.lastUpdate = millis();
  filteredMessageCount++;  
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeAlmInfo(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  almInfo.singleOverpressure = getBits(msg.data, 0, 2);
  almInfo.singleUnderpressure = getBits(msg.data, 2, 2);
  almInfo.pressureDiffTooLarge = getBits(msg.data, 8, 2);
  almInfo.dischargeOverflow = getBits(msg.data, 10, 2);
  almInfo.chargingOverflow = getBits(msg.data, 12, 2);
  almInfo.hyperpyrexia = getBits(msg.data, 14, 2);
  almInfo.tooLowTemperature = getBits(msg.data, 16, 2);
  almInfo.socTooLow = getBits(msg.data, 20, 2);
  almInfo.internalCommFailure = getBits(msg.data, 28, 2);
  
  uint16_t rawChg = getIntel16(msg.data, 4);
  uint16_t rawDchg = getIntel16(msg.data, 6);
  
  almInfo.chgEngCons = rawChg * 0.05;
  almInfo.dchgEngCons = rawDchg * 0.05;
  
  almInfo.valid = true;
  almInfo.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeBms6(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  int16_t rawCdcl = (int16_t)getIntel16(msg.data, 0);
  uint16_t rawCccl = getIntel16(msg.data, 2);
  int16_t rawPdcl = (int16_t)getIntel16(msg.data, 4);
  uint16_t rawPccl = getIntel16(msg.data, 6);
  
  bms6.cdcl = rawCdcl * 0.05 - 600;
  bms6.cccl = rawCccl * 0.05;
  bms6.pdcl = rawPdcl * 0.05 - 600;
  bms6.pccl = rawPccl * 0.05;
  
  bms6.valid = true;
  bms6.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeBattSt2(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  battSt2.capRemain = getIntel16(msg.data, 0) * 0.1;
  battSt2.fulChargeCap = getIntel16(msg.data, 2) * 0.1;
  battSt2.cycleCap = getIntel16(msg.data, 4) * 0.1;
  battSt2.cycleCount = getIntel16(msg.data, 6);
  
  battSt2.valid = true;
  battSt2.lastUpdate = millis();
  filteredMessageCount++;  
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeAllTemp(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  allTemp.tempMaskCode = msg.data[0];
  allTemp.cellTemp1 = (int8_t)msg.data[1] - 50;
  allTemp.cellTemp2 = (int8_t)msg.data[2] - 50;
  allTemp.cellTemp3 = (int8_t)msg.data[3] - 50;
  allTemp.cellTemp4 = (int8_t)msg.data[4] - 50;
  allTemp.cellTemp5 = (int8_t)msg.data[5] - 50;
  
  allTemp.valid = true;
  allTemp.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeBmsErrInfo(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  bmsErrInfo.excessLineResistance = getBit(msg.data, 0);
  bmsErrInfo.mosTooWarm = getBit(msg.data, 1);
  bmsErrInfo.cellCountDiscrepancy = getBit(msg.data, 2);
  bmsErrInfo.currentSensorAnomaly = getBit(msg.data, 3);
  bmsErrInfo.cellOvervoltage = getBit(msg.data, 4);
  bmsErrInfo.totalBatteryOverpressure = getBit(msg.data, 5);
  bmsErrInfo.chargeOvercurrent = getBit(msg.data, 6);
  bmsErrInfo.chargingShortCircuit = getBit(msg.data, 7);
  bmsErrInfo.chargingTempTooHigh = getBit(msg.data, 8);
  bmsErrInfo.chargingTempTooLow = getBit(msg.data, 9);
  bmsErrInfo.bmsInternalCommAnomaly = getBit(msg.data, 10);
  bmsErrInfo.singleUnderpressure = getBit(msg.data, 11);
  bmsErrInfo.totalBatteryUnderpressure = getBit(msg.data, 12);
  bmsErrInfo.dischargeOverflow = getBit(msg.data, 13);
  bmsErrInfo.dischargeShortCircuit = getBit(msg.data, 14);
  bmsErrInfo.dischargeTempTooHigh = getBit(msg.data, 15);
  bmsErrInfo.chargingMosFault = getBit(msg.data, 16);
  bmsErrInfo.dischargeMosFault = getBit(msg.data, 17);
  bmsErrInfo.faultCode = getBits(msg.data, 18, 8);
  bmsErrInfo.faultLevel = getBits(msg.data, 26, 3);
  
  bmsErrInfo.valid = true;
  bmsErrInfo.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeBmsInfo(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  bmsInfo.bmsRunTime = getIntel32(msg.data, 0);
  bmsInfo.heatCur = getIntel16(msg.data, 4);
  bmsInfo.soh = msg.data[6];
  
  bmsInfo.valid = true;
  bmsInfo.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeBmsSwSta(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  bmsSwSta.chgMosSta = getBit(msg.data, 0);
  bmsSwSta.dchgMosSta = getBit(msg.data, 1);
  bmsSwSta.balanSta = getBit(msg.data, 2);
  bmsSwSta.heatSta = getBit(msg.data, 3);
  bmsSwSta.chgDevPlugSta = getBit(msg.data, 4);
  bmsSwSta.accSta = getBit(msg.data, 5);
  
  bmsSwSta.valid = true;
  bmsSwSta.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeCellVoltage(const twai_message_t& msg, int startCell) {
  if (msg.data_length_code < 8) return;
  
  for (int i = 0; i < 4; i++) {
    int cellIdx = startCell + i - 1;
    if (cellIdx < 25) {
      cellVoltages.cellVoltages[cellIdx] = getIntel16(msg.data, i * 2);
    }
  }
  
  if (startCell + 3 > cellVoltages.cellCount) {
    cellVoltages.cellCount = startCell + 3;
  }
  cellVoltages.valid = true;
  cellVoltages.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeBmsChgInfo(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  bmsChgInfo.chgVol = getMotorola16(msg.data, 0) * 0.1;
  bmsChgInfo.chgCur = getMotorola16(msg.data, 2) * 0.1;
  bmsChgInfo.chgDevSw = (msg.data[4] & 0x80) != 0;
  bmsChgInfo.chgAndHeat = (msg.data[5] & 0x80) != 0;
  
  bmsChgInfo.valid = true;
  bmsChgInfo.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeCtrlInfo(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  ctrlInfo.maskCode = msg.data[0];
  ctrlInfo.chgSw = msg.data[1] != 0;
  ctrlInfo.dchgSw = msg.data[2] != 0;
  ctrlInfo.balanSw = msg.data[3] != 0;
  
  ctrlInfo.valid = true;
  ctrlInfo.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeMcuMsg1(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  mcuMsg1.dcVolt = msg.data[0];
  mcuMsg1.motorTemp = msg.data[1];
  mcuMsg1.cntrlTemp = msg.data[2];
  mcuMsg1.throttlePercent = msg.data[3];
  
  mcuMsg1.valid = true;
  mcuMsg1.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeMcuMsg2(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  mcuMsg2.speedMode = msg.data[0];
  mcuMsg2.motorSpeed = getMotorola16(msg.data, 1);
  mcuMsg2.motorSpdLim = getMotorola16(msg.data, 3);
  
  mcuMsg2.regenStatus = (msg.data[5] & 0x01) != 0;
  mcuMsg2.reverseStatus = (msg.data[5] & 0x02) != 0;
  mcuMsg2.forwardStatus = (msg.data[5] & 0x04) != 0;
  mcuMsg2.brakeStatus = (msg.data[5] & 0x08) != 0;
  mcuMsg2.cruiseStatus = (msg.data[5] & 0x10) != 0;
  
  mcuMsg2.motorStallFault = (msg.data[6] & 0x01) != 0;
  mcuMsg2.motorControllerFail = (msg.data[6] & 0x02) != 0;
  mcuMsg2.motorFailure = (msg.data[6] & 0x04) != 0;
  
  mcuMsg2.faultMotorPhase = (msg.data[7] & 0x01) != 0;
  mcuMsg2.faultMotorPosition = (msg.data[7] & 0x02) != 0;
  mcuMsg2.faultThrottle = (msg.data[7] & 0x04) != 0;
  mcuMsg2.faultUndervoltage = (msg.data[7] & 0x08) != 0;
  mcuMsg2.faultOvervoltage = (msg.data[7] & 0x10) != 0;
  mcuMsg2.faultOvertemperature = (msg.data[7] & 0x20) != 0;
  mcuMsg2.faultOvercurrent = (msg.data[7] & 0x40) != 0;
  mcuMsg2.faultMosfetProtection = (msg.data[7] & 0x80) != 0;
  
  mcuMsg2.valid = true;
  mcuMsg2.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeMcuMsg3(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  mcuMsg3.productId = getMotorola16(msg.data, 0);
  mcuMsg3.serialId = getMotorola32(msg.data, 2);
  mcuMsg3.fwVersion = getMotorola16(msg.data, 6);
  
  mcuMsg3.valid = true;
  mcuMsg3.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeMcuMsg4(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  mcuMsg4.phaseCurrent = getMotorola16(msg.data, 0) * 0.1;
  mcuMsg4.tripDistance = getMotorola32(msg.data, 2);
  
  uint16_t userFeatures = getMotorola16(msg.data, 6);
  mcuMsg4.direction = (userFeatures & 0x0001) != 0;
  mcuMsg4.modeA = (userFeatures & 0x0002) != 0;
  mcuMsg4.modeB = (userFeatures & 0x0004) != 0;
  mcuMsg4.modeC = (userFeatures & 0x0008) != 0;
  mcuMsg4.modeD = (userFeatures & 0x0010) != 0;
  mcuMsg4.cruise = (userFeatures & 0x0020) != 0;
  mcuMsg4.brake = (userFeatures & 0x0040) != 0;
  mcuMsg4.hillAssist = (userFeatures & 0x0100) != 0;
  mcuMsg4.regen = (userFeatures & 0x0200) != 0;
  
  mcuMsg4.valid = true;
  mcuMsg4.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeMcuMsg5(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  mcuMsg5.obdCode = getMotorola16(msg.data, 0);
  
  uint16_t mbist = getMotorola16(msg.data, 2);
  mcuMsg5.mbistMotorParams = (mbist & 0x0001) != 0;
  mcuMsg5.mbistDevLock = (mbist & 0x0002) != 0;
  mcuMsg5.mbistPhCurOffset = (mbist & 0x0004) != 0;
  mcuMsg5.mbistPosSensor = (mbist & 0x0008) != 0;
  mcuMsg5.mbistHwOc = (mbist & 0x0010) != 0;
  mcuMsg5.mbistSwOc = (mbist & 0x0020) != 0;
  mcuMsg5.mbistMtrCtrlLoop = (mbist & 0x0040) != 0;
  mcuMsg5.mbistCurSetpoint = (mbist & 0x0080) != 0;
  mcuMsg5.mbistMtrPhImb = (mbist & 0x0100) != 0;
  mcuMsg5.mbistMtrStall = (mbist & 0x0200) != 0;
  mcuMsg5.mbistWatchdog = (mbist & 0x0400) != 0;
  mcuMsg5.mbistLoopSat = (mbist & 0x0800) != 0;
  mcuMsg5.mbistLoopOscil = (mbist & 0x1000) != 0;
  
  uint16_t abist = getMotorola16(msg.data, 4);
  mcuMsg5.abistFeaParam = (abist & 0x0001) != 0;
  mcuMsg5.abistThrottle = (abist & 0x0002) != 0;
  mcuMsg5.abistBrdOt = (abist & 0x0004) != 0;
  mcuMsg5.abistMtrOt = (abist & 0x0008) != 0;
  mcuMsg5.abistLnOv = (abist & 0x0010) != 0;
  mcuMsg5.abistLnUv = (abist & 0x0020) != 0;
  
  mcuMsg5.valid = true;
  mcuMsg5.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeMcuMsg6(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  mcuMsg6.deratingStatus = msg.data[0];
  
  mcuMsg6.valid = true;
  mcuMsg6.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeAuxMotor1(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  auxMotor1.torqueEst = getIntel16(msg.data, 0) * 0.1;
  auxMotor1.speedEst = getIntel16(msg.data, 2);
  auxMotor1.controllerTemp = msg.data[4];
  auxMotor1.motorTemp = msg.data[6];
  
  auxMotor1.valid = true;
  auxMotor1.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeAuxMotor2(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  auxMotor2.motorCurrent = getIntel16(msg.data, 0) * 0.1;
  auxMotor2.motorVoltage = getIntel16(msg.data, 2) * 0.1;
  auxMotor2.mcuVoltage = getIntel16(msg.data, 4) * 0.1;
  auxMotor2.canLife = (msg.data[7] & 0x0F);
  
  auxMotor2.valid = true;
  auxMotor2.lastUpdate = millis();
  filteredMessageCount++;  
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeAuxMotor3(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  auxMotor3.igbtFault = getBit(msg.data, 0);
  auxMotor3.hwOcFault = getBit(msg.data, 1);
  auxMotor3.hwOvFault = getBit(msg.data, 2);
  auxMotor3.encoderFault = getBit(msg.data, 3);
  auxMotor3.lackPhaseFault = getBit(msg.data, 4);
  auxMotor3.currentDetectFault = getBit(msg.data, 5);
  auxMotor3.controllerUvFault = getBit(msg.data, 6);
  auxMotor3.controlOlLFault = getBit(msg.data, 7);
  auxMotor3.controlOlHFault = getBit(msg.data, 8);
  auxMotor3.overSpeedFault = getBit(msg.data, 9);
  auxMotor3.canOfflineFault = getBit(msg.data, 10);
  auxMotor3.igbtOtLFault = getBit(msg.data, 11);
  auxMotor3.igbtOtHFault = getBit(msg.data, 12);
  auxMotor3.motorOtLFault = getBit(msg.data, 13);
  auxMotor3.motorOtHFault = getBit(msg.data, 14);
  auxMotor3.uUpIgbtFault = getBit(msg.data, 15);
  auxMotor3.uDownIgbtFault = getBit(msg.data, 16);
  auxMotor3.vUpIgbtFault = getBit(msg.data, 17);
  auxMotor3.vDownIgbtFault = getBit(msg.data, 18);
  auxMotor3.wUpIgbtFault = getBit(msg.data, 19);
  auxMotor3.wDownIgbtFault = getBit(msg.data, 20);
  auxMotor3.hUOcFault = getBit(msg.data, 21);
  auxMotor3.hVOcFault = getBit(msg.data, 22);
  auxMotor3.hWOcFault = getBit(msg.data, 23);
  auxMotor3.igbtTempSensorFault = getBit(msg.data, 24);
  auxMotor3.motorTempSensorFault = getBit(msg.data, 25);
  auxMotor3.controllerUvWarning = getBit(msg.data, 26);
  auxMotor3.speedOvWarning = getBit(msg.data, 27);
  auxMotor3.motorStalling = getBit(msg.data, 40);
  
  auxMotor3.faultGrade = getBits(msg.data, 28, 4);
  auxMotor3.faultNum = getBits(msg.data, 32, 8);
  
  auxMotor3.valid = true;
  auxMotor3.lastUpdate = millis();
  filteredMessageCount++;   
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}

void decodeChrgOut(const twai_message_t& msg) {
  if (msg.data_length_code < 8) return;
  
  chrgOut.chargerVoltageOut = getMotorola16(msg.data, 0) * 0.1;
  chrgOut.chargerCurrentOut = getMotorola16(msg.data, 2) * 0.1;
  
  chrgOut.hardwareError = (msg.data[4] & 0x01) != 0;
  chrgOut.chargerTempFault = (msg.data[4] & 0x02) != 0;
  chrgOut.inputVoltageFault = (msg.data[4] & 0x04) != 0;
  chrgOut.workingCondition = (msg.data[4] & 0x08) != 0;
  chrgOut.commCondition = (msg.data[4] & 0x10) != 0;
  chrgOut.outputVoltageFault = (msg.data[4] & 0x20) != 0;
  chrgOut.outputOverCurrent = (msg.data[4] & 0x40) != 0;
  chrgOut.outputShortCircuit = (msg.data[4] & 0x80) != 0;
  
  chrgOut.chargerInputAcVolt = getMotorola16(msg.data, 5) * 0.1;
  chrgOut.chargerInternalTemp = (int8_t)msg.data[7] - 40;
  
  chrgOut.valid = true;
  chrgOut.lastUpdate = millis();
  filteredMessageCount++;   // <-- ADDED
  lastFilteredTime = millis();
  if (sessionState == SESSION_STATE_WAITING) {
      startNewSession();
      createNewLogFile();
      sessionState = SESSION_STATE_ACTIVE;
  }
}