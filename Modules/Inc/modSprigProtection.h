/*
	Sprig BMS protections for the series-hybrid backup battery
	(Sprig-Flight-Director docs/architecture/sprig-bms-can-v1.md, "Hold policy", D8 and D10-D14).

	Pure logic with no HAL dependency. Called once per main loop with the latest measurements.
	- Over-voltage, discharge over-current, charge over-current and over-temperature
	  (faults_a bits 0, 2, 3, 4) trip after a delay and stay latched until power-off (D11).
	- Soft and hard under-voltage and discharge under-temperature only block closing;
	  while energized they warn or hold (D12).
	- Charge under-temperature (bit 5) warns and drops the charge limit to 0.
	- Unknown cell or current data (faults_a bits 6 and 7) leaves the matching checks blind (D10).
 */

#ifndef MODSPRIGPROTECTION_H_
#define MODSPRIGPROTECTION_H_

#include <stdint.h>
#include <stdbool.h>
#include "libSprigCan.h"

typedef struct {
	float    cellHardOverVoltage;
	float    packHardOverVoltage;
	float    cellHardUnderVoltage;
	float    cellSoftUnderVoltage;
	uint32_t voltageTripDelayMs;          // Over-voltage must persist this long
	float    dischargeTripCurrent;
	uint32_t dischargeTripDelayMs;
	float    chargeTripCurrent;
	uint32_t chargeTripDelayMs;
	float    tempBatteryDischargeMax;     // Over-temperature (bit 4)
	float    tempBatteryDischargeMin;     // Discharge under-temperature: blocks closing only
	float    tempBatteryChargeMin;        // Charge under-temperature (bit 5): charge limit 0
	float    tempBMSMax;
	bool     tempBMSEnabled;
	uint32_t temperatureTripDelayMs;
} modSprigProtectionConfigTypedef;

typedef struct {
	uint32_t nowMs;
	bool     cellsKnown;                  // PEC-clean cell data and no cell-monitor fault
	float    cellVoltageHigh;
	float    cellVoltageLow;
	bool     packKnown;                   // Pack voltage and current sensor healthy
	float    packVoltage;
	float    packCurrent;                 // ENNOID convention: positive while charging
	float    tempBatteryHigh;
	float    tempBatteryLow;
	float    tempBMSHigh;
	bool     cellMonitorFault;            // faults_a bit 6
	bool     currentSensorFault;          // faults_a bit 7
} modSprigProtectionInputsTypedef;

typedef struct {
	bool     timing;
	uint32_t startMs;
} modSprigProtectionTimerTypedef;

typedef struct {
	modSprigProtectionTimerTypedef overVoltage;
	modSprigProtectionTimerTypedef dischargeOverCurrent;
	modSprigProtectionTimerTypedef chargeOverCurrent;
	modSprigProtectionTimerTypedef overTemperature;
	uint8_t tripped;                      // Latched faults_a bits 0, 2, 3, 4
} modSprigProtectionStateTypedef;

typedef struct {
	uint8_t faultsA;                      // Live bits 1, 5, 6, 7 plus the latched trip bits
	bool    trip;                         // A bit 0, 2, 3 or 4 fault has tripped: open and latch FAULT
	bool    lowSideBlock;                 // Soft/hard under-voltage or discharge under-temperature: will not close
	bool    chargeInhibit;                // Charge under-temperature: BMS_LIMITS charge limit 0
} modSprigProtectionResultTypedef;

#define SPRIG_PROTECTION_TRIP_BITS (SPRIG_FAULT_A_CELL_OVER_VOLTAGE | SPRIG_FAULT_A_DISCHARGE_OVER_CURRENT | \
                                    SPRIG_FAULT_A_CHARGE_OVER_CURRENT | SPRIG_FAULT_A_OVER_TEMPERATURE)

void modSprigProtectionInit(modSprigProtectionStateTypedef *state);
modSprigProtectionResultTypedef modSprigProtectionEvaluate(modSprigProtectionStateTypedef *state, const modSprigProtectionConfigTypedef *config, const modSprigProtectionInputsTypedef *in);

#endif
