#ifndef TYPES_H
#define TYPES_H

#include <SD.h>

// ================ FILE TYPE DEFINITIONS ================
typedef enum {
  FILE_TYPE_DATA,           // Normal data logging
  FILE_TYPE_DIAG_DISCONNECT, // ECU disconnect event
  FILE_TYPE_DIAG_RECOVERY,   // System recovery
  FILE_TYPE_DIAG_ERROR,      // Error condition
  FILE_TYPE_DIAG_BUSOFF,     // Bus-off event
  FILE_TYPE_SYSTEM,          // System logs
  FILE_TYPE_TEMP             // Temporary files
} FileType_t;

// ================ ECU STATE DEFINITIONS ================
typedef enum {
  ECU_STATE_UNKNOWN,
  ECU_STATE_CONNECTED,
  ECU_STATE_DISCONNECTED,
  ECU_STATE_DEGRADED,
  ECU_STATE_SILENT           // Connected but no messages (ECU off)
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
  bool isExtended;  // true for extended (29-bit), false for standard (11-bit)
  const char* name; // Message name for debugging
  uint16_t timeoutMs; // Timeout in milliseconds (0 = no timeout)
} CanFilter_t;

// ================ SESSION METADATA STRUCTURE ================
typedef struct {
  uint32_t sessionId;
  uint32_t fileSequence;
  time_t startEpoch;          // Unix timestamp (seconds since 1970-01-01 UTC)
  time_t endEpoch;            // Unix timestamp (seconds since 1970-01-01 UTC)
  char fileName[64];
  uint32_t sessionRecordCount;
  uint32_t fileRecordCount;
  ECUState_t ecuState;
  RotateReason_t rotateReason;
  uint32_t fileSize;
  bool cleanClosure;
} SessionMetadata_t;

// ================ BATTERY DATA STRUCTURES ================

// BATT_ST1 (0x2F4) - Battery pack data
typedef struct {
  float voltage;          // BattVolt in Volts
  float current;          // BattCurr in Amps
  uint8_t soc;            // SOC in percentage
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BattSt1_t;

// CELL_VOLT (0x4F4) - Cell voltage data
typedef struct {
  uint16_t maxCellVolt;   // MaxCellVolt in mV
  uint8_t maxCellNo;      // Max cell position
  uint16_t minCellVolt;   // MinCellVolt in mV
  uint8_t minCellNo;      // Min cell position
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CellVolt_t;

// CELL_TEMP (0x5F4) - Cell temperature data
typedef struct {
  int8_t maxCellTemp;     // Max cell temperature in °C
  uint8_t maxCtNO;        // Max temperature cell position
  int8_t minCellTemp;     // Min cell temperature in °C
  uint8_t minCtNO;        // Min temperature cell position
  int8_t avgCellTemp;     // Average cell temperature in °C
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CellTemp_t;

// ALM_INFO (0x7F4) - Alarm information
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
  float chgEngCons;        // Charging energy consumption in KWh
  float dchgEngCons;       // Discharging energy consumption in KWh
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AlmInfo_t;

// BMS_6 (0x08F4) - Current limits
typedef struct {
  float cdcl;     // Continuous Discharging Current Limit in A
  float cccl;     // Continuous Charging Current Limit in A
  float pdcl;     // Peak Discharging Current Limit in A
  float pccl;     // Peak Charging Current Limit in A
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} Bms6_t;

// BATT_ST2 (0x18F128F4) - Battery stats
typedef struct {
  float capRemain;        // Remaining capacity in AH
  float fulChargeCap;     // Full charge capacity in AH
  float cycleCap;         // Cycle capacity in AH
  uint16_t cycleCount;    // Number of battery cycles
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BattSt2_t;

// ALL_TEMP (0x18F228F4) - All temperatures
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

// BMSERR_INFO (0x18F328F4) - Error information
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

// BMS_INFO (0x18F428F4) - BMS info
typedef struct {
  uint32_t bmsRunTime;     // BMS runtime in seconds
  uint16_t heatCur;        // Heater current in mA
  uint8_t soh;             // State of Health in %
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BmsInfo_t;

// BmsSwSta (0x18F528F4) - Switch states
typedef struct {
  bool chgMosSta;          // Charge MOS status
  bool dchgMosSta;         // Discharge MOS status
  bool balanSta;           // Balancing status
  bool heatSta;            // Heater status
  bool chgDevPlugSta;      // Charger plugged status
  bool accSta;             // ACC status
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BmsSwSta_t;

// CellVol_x (0x18Ex28F4) - Individual cell voltages (up to 25 cells)
typedef struct {
  uint16_t cellVoltages[25];  // Cell voltages in mV (index 0 = cell 1)
  uint8_t cellCount;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CellVoltages_t;

// BMSChgINFO (0x1806E5F4) - Charging info
typedef struct {
  float chgVol;            // Charging voltage in V
  float chgCur;            // Charging current in A
  bool chgDevSw;           // Charger switch
  bool chgAndHeat;         // Charging/heating mode
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} BmsChgInfo_t;

// Ctrl_INFO (0x18F0F428) - Control info
typedef struct {
  uint8_t maskCode;
  bool chgSw;              // Charge switch
  bool dchgSw;             // Discharge switch
  bool balanSw;            // Balancing switch
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} CtrlInfo_t;

// MCU_MSG_1 (0x102200A0)
typedef struct {
  uint8_t dcVolt;          // DC Voltage in V
  uint8_t motorTemp;       // Motor temperature in °C
  uint8_t cntrlTemp;       // Controller temperature in °C
  uint8_t throttlePercent; // Throttle percentage
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg1_t;

// MCU_MSG_2 (0x102200A1)
typedef struct {
  uint8_t speedMode;
  uint16_t motorSpeed;     // Motor speed in RPM
  uint16_t motorSpdLim;    // Motor speed limit
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

// MCU_MSG_3 (0x102200A2)
typedef struct {
  uint16_t productId;
  uint32_t serialId;
  uint16_t fwVersion;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg3_t;

// MCU_MSG_4 (0x102200A3)
typedef struct {
  float phaseCurrent;      // Phase current in A
  uint32_t tripDistance;   // Trip distance in km
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

// MCU_MSG_5 (0x102200A4)
typedef struct {
  uint16_t obdCode;
  // MBIST faults
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
  // ABIST faults
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

// MCU_MSG_6 (0x102200A5)
typedef struct {
  uint8_t deratingStatus;
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} McuMsg6_t;

// AUX_MOTOR_1 (0x19FF50F0)
typedef struct {
  float torqueEst;         // Estimated torque in Nm
  uint16_t speedEst;       // Estimated speed in RPM
  uint8_t controllerTemp;  // Controller temperature in °C
  uint8_t motorTemp;       // Motor temperature in °C
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AuxMotor1_t;

// AUX_MOTOR_2 (0x19FF50F1)
typedef struct {
  float motorCurrent;      // Motor current in A
  float motorVoltage;      // Motor voltage in V
  float mcuVoltage;        // MCU voltage in V
  uint8_t canLife;         // CAN life counter
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} AuxMotor2_t;

// AUX_MOTOR_3 (0x19FF50F2)
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

// CHRG_OUT (0x18FF50E5)
typedef struct {
  float chargerVoltageOut;   // Charger output voltage in V
  float chargerCurrentOut;   // Charger output current in A
  bool hardwareError;
  bool chargerTempFault;
  bool inputVoltageFault;
  bool workingCondition;
  bool commCondition;
  bool outputVoltageFault;
  bool outputOverCurrent;
  bool outputShortCircuit;
  float chargerInputAcVolt;  // AC input voltage in V
  int8_t chargerInternalTemp; // Internal temperature in °C
  bool valid;
  unsigned long lastUpdate;
  unsigned long timeoutMs;
} ChrgOut_t;

// ================ DBC STRUCTURES ================
struct DBCSignal {
    String name;
    uint8_t startBit;
    uint8_t length;
    bool isIntel;          // true = Intel, false = Motorola
    float scale;
    float offset;
    float minVal;
    float maxVal;
    String unit;
    bool isSigned;         // true if signal is signed (two's complement)
    bool isSelected;       // for UI
    String receiver;       // Added: receiver node (e.g., "CLUSTER")
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