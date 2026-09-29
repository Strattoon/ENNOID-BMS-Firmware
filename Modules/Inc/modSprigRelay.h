/*
	Sprig relay supervisor: the "Sequence" and "Hold policy" of the Sprig BMS
	CAN protocol v1 (Sprig-Flight-Director docs/architecture/sprig-bms-can-v1.md).

	Pure logic with no HAL dependency. The caller samples every input, calls
	modSprigRelayStep() once per main loop and drives the outputs it returns.
 */

#ifndef MODSPRIGRELAY_H_
#define MODSPRIGRELAY_H_

#include <stdint.h>
#include <stdbool.h>
#include "libSprigCan.h"

typedef enum {
	SPRIG_PHASE_STANDBY = 0,
	SPRIG_PHASE_PRECHARGING,
	SPRIG_PHASE_ENERGIZED,
	SPRIG_PHASE_PRECHARGE_FAILED
} modSprigRelayPhaseTypedef;

typedef struct {
	uint32_t relayRequestDebounceMs;
	uint32_t prechargeTimeoutMs;
	float    prechargeMatchFraction;
	float    prechargeCrossCheckVolts;
} modSprigRelayConfigTypedef;

typedef struct {
	uint32_t nowMs;
	bool     configValid;
	bool     maintenance;           // Blocks closing; relays are never closed in MAINTENANCE
	bool     protectionsOk;         // Discharge allowed and pack in SOA (permissions bits 0 and 2), faults_a bits 0-5 clear
	bool     sensingFault;          // faults_a bit 6 or 7: blocks closing, holds while energized (D10)
	bool     dtiWatchHealthy;       // 0x20, 0x22 and 0x24 fresh from the configured node
	bool     dtiCanRequest;         // 0x24 digital output (only meaningful while the watch is healthy)
	uint8_t  dtiFaultCode;          // 0x22 fault code
	float    dtiInputVoltage;       // 0x20 input voltage
	bool     expanderOk;            // Expander responds and reads back the strap pattern
	bool     hwRequest;             // Hardwired DTI digital output (only meaningful while the expander is OK)
	bool     hvilClosed;            // HVIL loop (only meaningful while the expander is OK)
	float    loadVoltage;           // Master-HV isolated load-side voltage
	float    packVoltage;
} modSprigRelayInputsTypedef;

typedef struct {
	bool precharge;
	bool main;
	bool dtiEnable;                 // "Main closed and precharge complete"
} modSprigRelayOutputsTypedef;

typedef struct {
	modSprigRelayPhaseTypedef phase;
	uint8_t  faultsB;
	uint8_t  lastOpenReason;
	uint32_t prechargeStartMs;
	uint32_t disagreementStartMs;
	bool     disagreementTiming;
	bool     prechargeMismatchSeen;
} modSprigRelayStateTypedef;

void modSprigRelayInit(modSprigRelayStateTypedef *state);
modSprigRelayOutputsTypedef modSprigRelayStep(modSprigRelayStateTypedef *state, const modSprigRelayConfigTypedef *config, const modSprigRelayInputsTypedef *in);

// The operational state machine opened the relays outside the supervisor (hard protection, power button, maintenance).
void modSprigRelayForceOpen(modSprigRelayStateTypedef *state, sprigCanOpenReasonTypedef reason);

#endif
