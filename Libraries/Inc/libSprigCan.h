/*
	Sprig BMS CAN protocol v1 frame codec.

	Spec: Sprig-Flight-Director docs/architecture/sprig-bms-can-v1.md.
	Pure C with no HAL dependency, so the same code runs in the firmware and in
	the host unit tests (tests/sprig).
 */

#ifndef LIBSPRIGCAN_H_
#define LIBSPRIGCAN_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SPRIG_CAN_DLC                   8
#define SPRIG_CAN_PROTOCOL_VERSION      1
#define SPRIG_CAN_UNUSED_BYTE           0xFF

#define SPRIG_CAN_ID_STATE              0x500
#define SPRIG_CAN_ID_PACK               0x501
#define SPRIG_CAN_ID_LIMITS             0x502
#define SPRIG_CAN_ID_CELLS              0x503
#define SPRIG_CAN_ID_TEMPS              0x504
#define SPRIG_CAN_ID_ENERGY             0x505
#define SPRIG_CAN_ID_IDENT              0x50F
// 0x50A is the CANopen RPDO4 COB-ID of charger node 0x0A: never transmit on it.
#define SPRIG_CAN_ID_FORBIDDEN          0x50A

#define SPRIG_CAN_BUS_VOLTAGE_UNAVAILABLE 0xFFFF
#define SPRIG_CAN_CELL_INDEX_UNKNOWN      0xFF

// BMS_STATE.state (byte 0)
typedef enum {
	SPRIG_STATE_INIT = 0,
	SPRIG_STATE_STANDBY,
	SPRIG_STATE_PRECHARGING,
	SPRIG_STATE_ENERGIZED,
	SPRIG_STATE_CHARGING,
	SPRIG_STATE_BALANCING,
	SPRIG_STATE_PRECHARGE_FAILED,
	SPRIG_STATE_FAULT,
	SPRIG_STATE_POWER_DOWN,
	SPRIG_STATE_MAINTENANCE,
	SPRIG_STATE_FORCED_ON
} sprigCanStateTypedef;

// BMS_STATE.relay_outputs (byte 1)
#define SPRIG_RELAY_PRECHARGE           (1u << 0)
#define SPRIG_RELAY_DISCHARGE           (1u << 1)
#define SPRIG_RELAY_CHARGE              (1u << 2)
#define SPRIG_RELAY_DISCHARGE_NEGATIVE  (1u << 3)  // Master-HV SWITCH_DISCHARGEHV, PC15 (D9); bits 4-7 reserved 0

// BMS_STATE.permissions (byte 2)
#define SPRIG_PERM_DISCHARGE_ALLOWED    (1u << 0)
#define SPRIG_PERM_CHARGE_ALLOWED       (1u << 1)
#define SPRIG_PERM_IN_SOA               (1u << 2)
#define SPRIG_PERM_DTI_WATCH_HEALTHY    (1u << 3)
#define SPRIG_PERM_CAN_REQUEST          (1u << 4)
#define SPRIG_PERM_POWER_BUTTON         (1u << 5)
#define SPRIG_PERM_BALANCING            (1u << 6)
#define SPRIG_PERM_HW_REQUEST           (1u << 7)

// BMS_STATE.faults_a (byte 3), latched protection faults
#define SPRIG_FAULT_A_CELL_OVER_VOLTAGE      (1u << 0)
#define SPRIG_FAULT_A_CELL_UNDER_VOLTAGE     (1u << 1)
#define SPRIG_FAULT_A_DISCHARGE_OVER_CURRENT (1u << 2)
#define SPRIG_FAULT_A_CHARGE_OVER_CURRENT    (1u << 3)
#define SPRIG_FAULT_A_OVER_TEMPERATURE       (1u << 4)
#define SPRIG_FAULT_A_CHARGE_UNDER_TEMP      (1u << 5)
#define SPRIG_FAULT_A_CELL_MONITOR_COMM      (1u << 6)
#define SPRIG_FAULT_A_CURRENT_SENSOR         (1u << 7)

// BMS_STATE.faults_b (byte 4), latched system faults
#define SPRIG_FAULT_B_PRECHARGE_TIMEOUT      (1u << 0)
#define SPRIG_FAULT_B_PRECHARGE_MISMATCH     (1u << 1)
#define SPRIG_FAULT_B_DTI_WATCH_TIMEOUT      (1u << 2)
#define SPRIG_FAULT_B_DTI_FAULT              (1u << 3)
#define SPRIG_FAULT_B_CONFIG_INVALID         (1u << 4)
#define SPRIG_FAULT_B_HVIL_OPEN              (1u << 5)
#define SPRIG_FAULT_B_REQUEST_DISAGREEMENT   (1u << 6)
#define SPRIG_FAULT_B_EXPANDER               (1u << 7)

// BMS_STATE.last_open_reason (byte 5)
typedef enum {
	SPRIG_OPEN_NONE = 0,
	SPRIG_OPEN_REQUEST_WITHDRAWN,
	SPRIG_OPEN_PROTECTION,
	SPRIG_OPEN_DTI_WATCH_LOST,
	SPRIG_OPEN_PRECHARGE_FAILURE,
	SPRIG_OPEN_POWER_BUTTON,
	SPRIG_OPEN_MAINTENANCE
} sprigCanOpenReasonTypedef;

// BMS_IDENT.hardware_id
#define SPRIG_HARDWARE_ID_GEN1_LTC6811  1
#define SPRIG_HARDWARE_ID_GEN1_LTC6813  2

typedef struct {
	uint8_t state;
	uint8_t relayOutputs;
	uint8_t permissions;
	uint8_t faultsA;
	uint8_t faultsB;
	uint8_t lastOpenReason;
} libSprigCanStateFrameTypedef;

typedef struct {
	uint16_t packVoltageDeciVolt;
	int16_t  packCurrentDeciAmp;      // Positive = discharge (current leaving the pack)
	uint16_t busVoltageDeciVolt;      // SPRIG_CAN_BUS_VOLTAGE_UNAVAILABLE when unknown
} libSprigCanPackFrameTypedef;

typedef struct {
	uint16_t dischargeCurrentLimitDeciAmp;
	uint16_t chargeCurrentLimitDeciAmp;
	uint16_t stateOfChargeCentiPercent; // 0..10000
} libSprigCanLimitsFrameTypedef;

typedef struct {
	uint16_t cellVoltageMinMilliVolt;
	uint16_t cellVoltageMaxMilliVolt;
	uint8_t  cellIndexMin;              // 0-based, SPRIG_CAN_CELL_INDEX_UNKNOWN when unknown
	uint8_t  cellIndexMax;
} libSprigCanCellsFrameTypedef;

typedef struct {
	int16_t cellTempMaxDeciC;
	int16_t cellTempAvgDeciC;
	int8_t  bmsTempMaxC;
} libSprigCanTempsFrameTypedef;

typedef struct {
	uint16_t remainingCapacityCentiAh;
	uint16_t fullCapacityCentiAh;
	uint8_t  seriesCells;
} libSprigCanEnergyFrameTypedef;

typedef struct {
	uint32_t firmwareBuild;
	uint8_t  configRevision;
	uint8_t  hardwareId;
} libSprigCanIdentFrameTypedef;

// CRC-8/SAE-J1850: poly 0x1D, init 0xFF, xorout 0xFF, no reflection. Check("123456789") = 0x4B.
uint8_t  libSprigCanCrc8(const uint8_t *data, size_t length);

// Writes byte 6 (version and counter) and byte 7 (CRC over the big-endian ID then bytes 0-6).
void     libSprigCanFinalize(uint16_t id, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);

// Round to the nearest raw step and saturate to [min, max]. NaN maps to nanValue.
int32_t  libSprigCanScale(float value, float stepsPerUnit, int32_t min, int32_t max, int32_t nanValue);

void     libSprigCanPackState(const libSprigCanStateFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);
void     libSprigCanPackPack(const libSprigCanPackFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);
void     libSprigCanPackLimits(const libSprigCanLimitsFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);
void     libSprigCanPackCells(const libSprigCanCellsFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);
void     libSprigCanPackTemps(const libSprigCanTempsFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);
void     libSprigCanPackEnergy(const libSprigCanEnergyFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);
void     libSprigCanPackIdent(const libSprigCanIdentFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]);

// ---- DTI HV-550/850 status packets (DTI CAN manual V2.5, Flight Director dti_decoder.cpp) ----

typedef enum {
	SPRIG_DTI_FORMAT_STANDARD = 0,
	SPRIG_DTI_FORMAT_EXTENDED = 1,
	SPRIG_DTI_FORMAT_UNSET    = 0xFF
} libSprigDtiFormatTypedef;

#define SPRIG_DTI_NODE_UNSET            0xFF
#define SPRIG_DTI_STD_NODE_RESERVED     10  // 0x040A/0x048A collide with the ENNOID charger path

#define SPRIG_DTI_PACKET_GENERAL_1      0x20 // ERPM, duty, input voltage
#define SPRIG_DTI_PACKET_GENERAL_3      0x22 // Temperatures, fault code
#define SPRIG_DTI_PACKET_GENERAL_5      0x24 // Throttle, brake, digital I/O

typedef enum {
	SPRIG_DTI_NOT_MINE = 0,   // Other node, other format or not a watched packet
	SPRIG_DTI_REJECTED,       // Watched packet with a DLC other than 8
	SPRIG_DTI_DECODED
} libSprigDtiResultTypedef;

typedef struct {
	uint8_t packet;
	int16_t inputVoltageVolt;  // 0x20
	uint8_t faultCode;         // 0x22
	bool    digitalOutput;     // 0x24, the configured output
} libSprigDtiFrameTypedef;

bool libSprigDtiConfigValid(uint8_t format, uint8_t node);
uint32_t libSprigDtiId(uint8_t format, uint8_t packet, uint8_t node);
libSprigDtiResultTypedef libSprigDtiDecode(uint8_t format, uint8_t node, uint8_t digitalOutputNumber,
                                           uint32_t id, bool extended, uint8_t dlc, const uint8_t *data,
                                           libSprigDtiFrameTypedef *out);

#endif
