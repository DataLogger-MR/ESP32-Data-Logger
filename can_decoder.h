#ifndef CAN_DECODER_H
#define CAN_DECODER_H

#include <Arduino.h>
#include "driver/twai.h"
#include "types.h"
#include "config.h"

// ================ CAN FILTER DEFINITIONS ================
extern const CanFilter_t FILTERED_IDS[];
extern const int FILTER_COUNT;

// ================ GLOBAL DATA INSTANCES ================
extern BattSt1_t battSt1;
extern CellVolt_t cellVolt;
extern CellTemp_t cellTemp;
extern AlmInfo_t almInfo;
extern Bms6_t bms6;
extern BattSt2_t battSt2;
extern AllTemp_t allTemp;
extern BmsErrInfo_t bmsErrInfo;
extern BmsInfo_t bmsInfo;
extern BmsSwSta_t bmsSwSta;
extern CellVoltages_t cellVoltages;
extern BmsChgInfo_t bmsChgInfo;
extern CtrlInfo_t ctrlInfo;
extern McuMsg1_t mcuMsg1;
extern McuMsg2_t mcuMsg2;
extern McuMsg3_t mcuMsg3;
extern McuMsg4_t mcuMsg4;
extern McuMsg5_t mcuMsg5;
extern McuMsg6_t mcuMsg6;
extern AuxMotor1_t auxMotor1;
extern AuxMotor2_t auxMotor2;
extern AuxMotor3_t auxMotor3;
extern ChrgOut_t chrgOut;

// ================ FUNCTION PROTOTYPES ================
void initCAN();
void processCANMessages();
bool acceptCANId(uint32_t id, bool isExtended);
const char* getMessageName(uint32_t id, bool isExtended);
void initializeTimeouts();

void decodeBattSt1(const twai_message_t& msg);
void decodeCellVolt(const twai_message_t& msg);
void decodeCellTemp(const twai_message_t& msg);
void decodeAlmInfo(const twai_message_t& msg);
void decodeBms6(const twai_message_t& msg);
void decodeBattSt2(const twai_message_t& msg);
void decodeAllTemp(const twai_message_t& msg);
void decodeBmsErrInfo(const twai_message_t& msg);
void decodeBmsInfo(const twai_message_t& msg);
void decodeBmsSwSta(const twai_message_t& msg);
void decodeCellVoltage(const twai_message_t& msg, int startCell);
void decodeBmsChgInfo(const twai_message_t& msg);
void decodeCtrlInfo(const twai_message_t& msg);
void decodeMcuMsg1(const twai_message_t& msg);
void decodeMcuMsg2(const twai_message_t& msg);
void decodeMcuMsg3(const twai_message_t& msg);
void decodeMcuMsg4(const twai_message_t& msg);
void decodeMcuMsg5(const twai_message_t& msg);
void decodeMcuMsg6(const twai_message_t& msg);
void decodeAuxMotor1(const twai_message_t& msg);
void decodeAuxMotor2(const twai_message_t& msg);
void decodeAuxMotor3(const twai_message_t& msg);
void decodeChrgOut(const twai_message_t& msg);


extern twai_status_info_t twai_status;

#endif