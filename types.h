#ifndef TYPES_H
#define TYPES_H

#include <SD.h>

// ================ FILE TYPE DEFINITIONS ================
typedef enum {
  FILE_TYPE_DATA,           
  FILE_TYPE_DIAG_DISCONNECT, 
  FILE_TYPE_DIAG_RECOVERY,   
  FILE_TYPE_DIAG_ERROR,      
  FILE_TYPE_DIAG_BUSOFF,     
  FILE_TYPE_SYSTEM,          
  FILE_TYPE_TEMP             
} FileType_t;

// ================ ECU STATE DEFINITIONS ================
typedef enum {
  ECU_STATE_UNKNOWN,
  ECU_STATE_CONNECTED,
  ECU_STATE_DISCONNECTED,
  ECU_STATE_DEGRADED,
  ECU_STATE_SILENT           
} ECUState_t;

// ================ SESSION STATE DEFINITIONS ================
typedef enum {
  SESSION_STATE_BOOT,
  SESSION_STATE_INIT,
  SESSION_STATE_WAITING,
  SESSION_STATE_ACTIVE,
  SESSION_STATE_ROTATING,
  SESSION_STATE_CLOSING,
  SESSION_STATE_SHUTDOWN
} SessionState_t;

// ================ ROTATION REASON DEFINITIONS ================
typedef enum {
  ROTATE_REASON_NONE,
  ROTATE_REASON_HOURLY,
  ROTATE_REASON_SIZE,
  ROTATE_REASON_ECU_DISCONNECT,
  ROTATE_REASON_USER_COMMAND,
  ROTATE_REASON_SYSTEM,
  ROTATE_REASON_RECOVERY
} RotateReason_t;

// ================ CAN FILTER STRUCTURE ================
typedef struct {
  uint32_t id;
  bool isExtended;  
  const char* name; 
  uint16_t timeoutMs; 
} CanFilter_t;

// ================ SESSION METADATA STRUCTURE ================
typedef struct {
  uint32_t sessionId;
  uint32_t fileSequence;
  time_t startEpoch;          
  time_t endEpoch;            
  char fileName[64];
  uint32_t sessionRecordCount;
  uint32_t fileRecordCount;
  ECUState_t ecuState;
  RotateReason_t rotateReason;
  uint32_t fileSize;
  bool cleanClosure;
} SessionMetadata_t;

// ================ BATTERY DATA STRUCTURES ================

typedef struct {
  float voltage;         
  float current;          
  uint8_t soc;            
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BattSt1_t;

typedef struct {
  uint16_t maxCellVolt;   
  uint8_t maxCellNo;      
  uint16_t minCellVolt;   
  uint8_t minCellNo;      
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CellVolt_t;

typedef struct {
  int8_t maxCellTemp;     
  uint8_t maxCtNO;        
  int8_t minCellTemp;     
  uint8_t minCtNO;        
  int8_t avgCellTemp;     
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CellTemp_t;

typedef struct {
  uint8_t singleOverpressure;
  uint8_t singleUnderpressure;
  uint8_t pressureDiffTooLarge;
  uint8_t dischargeOverflow;
  uint8_t chargingOverflow;
  uint8_t hyperpyrexia;
  uint8_t tooLowTemperature;
  uint8_t socTooLow;
  uint8_t internalCommFailure;
  float chgEngCons;        
  float dchgEngCons;       
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AlmInfo_t;

typedef struct {
  float cdcl;     
  float cccl;     
  float pdcl;     
  float pccl;     
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} Bms6_t;

typedef struct {
  float capRemain;        
  float fulChargeCap;     
  float cycleCap;         
  uint16_t cycleCount;    
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BattSt2_t;

typedef struct {
  uint8_t tempMaskCode;
  int8_t cellTemp1;
  int8_t cellTemp2;
  int8_t cellTemp3;
  int8_t cellTemp4;
  int8_t cellTemp5;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AllTemp_t;

typedef struct {
  bool excessLineResistance;
  bool mosTooWarm;
  bool cellCountDiscrepancy;
  bool currentSensorAnomaly;
  bool cellOvervoltage;
  bool totalBatteryOverpressure;
  bool chargeOvercurrent;
  bool chargingShortCircuit;
  bool chargingTempTooHigh;
  bool chargingTempTooLow;
  bool bmsInternalCommAnomaly;
  bool singleUnderpressure;
  bool totalBatteryUnderpressure;
  bool dischargeOverflow;
  bool dischargeShortCircuit;
  bool dischargeTempTooHigh;
  bool chargingMosFault;
  bool dischargeMosFault;
  uint8_t faultCode;
  uint8_t faultLevel;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BmsErrInfo_t;

typedef struct {
  uint32_t bmsRunTime;     
  uint16_t heatCur;        
  uint8_t soh;             
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BmsInfo_t;

typedef struct {
  bool chgMosSta;          
  bool dchgMosSta;         
  bool balanSta;           
  bool heatSta;            
  bool chgDevPlugSta;      
  bool accSta;             
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BmsSwSta_t;

typedef struct {
  uint16_t cellVoltages[25];  
  uint8_t cellCount;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CellVoltages_t;

typedef struct {
  float chgVol;            
  float chgCur;            
  bool chgDevSw;           
  bool chgAndHeat;         
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BmsChgInfo_t;

typedef struct {
  uint8_t maskCode;
  bool chgSw;              
  bool dchgSw;             
  bool balanSw;            
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CtrlInfo_t;

// MCU_MSG_1 (0x102200A0)
typedef struct {
  uint8_t dcVolt;          
  uint8_t motorTemp;       
  uint8_t cntrlTemp;       
  uint8_t throttlePercent; 
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg1_t;

typedef struct {
  uint8_t speedMode;
  uint16_t motorSpeed;     
  uint16_t motorSpdLim;    
  bool regenStatus;
  bool reverseStatus;
  bool forwardStatus;
  bool brakeStatus;
  bool cruiseStatus;
  bool motorStallFault;
  bool motorControllerFail;
  bool motorFailure;
  bool faultMotorPhase;
  bool faultMotorPosition;
  bool faultThrottle;
  bool faultUndervoltage;
  bool faultOvervoltage;
  bool faultOvertemperature;
  bool faultOvercurrent;
  bool faultMosfetProtection;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg2_t;

typedef struct {
  uint16_t productId;
  uint32_t serialId;
  uint16_t fwVersion;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg3_t;

typedef struct {
  float phaseCurrent;      
  uint32_t tripDistance;   
  bool direction;
  bool modeA;
  bool modeB;
  bool modeC;
  bool modeD;
  bool cruise;
  bool brake;
  bool hillAssist;
  bool regen;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg4_t;

typedef struct {
  uint16_t obdCode;
  bool mbistMotorParams;
  bool mbistDevLock;
  bool mbistPhCurOffset;
  bool mbistPosSensor;
  bool mbistHwOc;
  bool mbistSwOc;
  bool mbistMtrCtrlLoop;
  bool mbistCurSetpoint;
  bool mbistMtrPhImb;
  bool mbistMtrStall;
  bool mbistWatchdog;
  bool mbistLoopSat;
  bool mbistLoopOscil;

  bool abistFeaParam;
  bool abistThrottle;
  bool abistBrdOt;
  bool abistMtrOt;
  bool abistLnOv;
  bool abistLnUv;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg5_t;

typedef struct {
  uint8_t deratingStatus;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg6_t;

typedef struct {
  float torqueEst;         
  uint16_t speedEst;       
  uint8_t controllerTemp;  
  uint8_t motorTemp;       
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AuxMotor1_t;

typedef struct {
  float motorCurrent;      
  float motorVoltage;     
  float mcuVoltage;       
  uint8_t canLife;         
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AuxMotor2_t;

typedef struct {
  bool igbtFault;
  bool hwOcFault;
  bool hwOvFault;
  bool encoderFault;
  bool lackPhaseFault;
  bool currentDetectFault;
  bool controllerUvFault;
  bool controlOlLFault;
  bool controlOlHFault;
  bool overSpeedFault;
  bool canOfflineFault;
  bool igbtOtLFault;
  bool igbtOtHFault;
  bool motorOtLFault;
  bool motorOtHFault;
  bool uUpIgbtFault;
  bool uDownIgbtFault;
  bool vUpIgbtFault;
  bool vDownIgbtFault;
  bool wUpIgbtFault;
  bool wDownIgbtFault;
  bool hUOcFault;
  bool hVOcFault;
  bool hWOcFault;
  bool igbtTempSensorFault;
  bool motorTempSensorFault;
  bool controllerUvWarning;
  bool speedOvWarning;
  bool motorStalling;
  uint8_t faultGrade;
  uint8_t faultNum;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AuxMotor3_t;

typedef struct {
  float chargerVoltageOut;   
  float chargerCurrentOut;   
  bool hardwareError;
  bool chargerTempFault;
  bool inputVoltageFault;
  bool workingCondition;
  bool commCondition;
  bool outputVoltageFault;
  bool outputOverCurrent;
  bool outputShortCircuit;
  float chargerInputAcVolt;  
  int8_t chargerInternalTemp; 
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} ChrgOut_t;

// ================ DBC STRUCTURES ================
struct DBCSignal {
    String name;
    uint8_t startBit;
    uint8_t length;
    bool isIntel;          
    float scale;
    float offset;
    float minVal;
    float maxVal;
    String unit;
    bool isSigned;         
    bool isSelected;       
    String receiver;       
};

struct DBCMessage {
    uint32_t id;
    bool isExtended;
    String name;
    uint8_t dlc;
    String transmitter;
    std::vector<DBCSignal> signals;
};

#endif