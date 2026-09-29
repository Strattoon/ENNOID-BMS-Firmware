/*
	Sprig BMS CAN protocol v1 on the ENNOID Gen 1 Master-HV
	(Sprig-Flight-Director docs/architecture/sprig-bms-can-v1.md).

	Active when generalConfig->sprigCanEnabled is set (forced by SPRIG_FLIGHT_BUILD).
 */

#ifndef MODSPRIG_H_
#define MODSPRIG_H_

#include <stdint.h>
#include <stdbool.h>
#include "modConfig.h"
#include "modPowerElectronics.h"
#include "modStateOfCharge.h"
#include "modSprigRelay.h"

#ifndef SPRIG_FIRMWARE_BUILD
#define SPRIG_FIRMWARE_BUILD 0x00000000u  // First 8 hex digits of the firmware commit, set by the makefile
#endif

void modSprigInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer);
bool modSprigEnabled(void);
bool modSprigMaintenance(void);
bool modSprigSetMaintenance(bool enable);   // Entering needs STANDBY with every relay output open
bool modSprigRelaysOpen(void);

// Main loop: inputs, fault latching and the 0x500-0x50F transmit schedule.
void modSprigTask(void);

// Interrupt context: every received CAN frame.
void modSprigCANReceive(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data);

// Called by the operational state machine in STANDBY, PRE_CHARGE, LOAD_ENABLED, ERROR_PRECHARGE and ERROR.
// Runs the relay supervisor, drives the relays and returns the phase.
modSprigRelayPhaseTypedef modSprigRelayTask(void);

// The operational state machine opened the relays itself (power button).
void modSprigRelaysForcedOpen(sprigCanOpenReasonTypedef reason);

// A faults_a bit 0, 2, 3 or 4 protection has tripped (latched until power-off, D11).
bool modSprigProtectionTripped(void);

#endif
