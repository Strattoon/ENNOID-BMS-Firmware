/*
	Sprig BMS protections. See modSprigProtection.h.
 */

#include "modSprigProtection.h"

void modSprigProtectionInit(modSprigProtectionStateTypedef *state) {
	state->overVoltage.timing          = false;
	state->dischargeOverCurrent.timing = false;
	state->chargeOverCurrent.timing    = false;
	state->overTemperature.timing      = false;
	state->tripped                     = 0;
}

// True once the condition has held continuously for delayMs.
static bool modSprigProtectionPersists(modSprigProtectionTimerTypedef *timer, bool condition, uint32_t nowMs, uint32_t delayMs) {
	if(!condition) {
		timer->timing = false;
		return false;
	}
	if(!timer->timing) {
		timer->timing  = true;
		timer->startMs = nowMs;
	}
	return (uint32_t)(nowMs - timer->startMs) >= delayMs;
}

modSprigProtectionResultTypedef modSprigProtectionEvaluate(modSprigProtectionStateTypedef *state, const modSprigProtectionConfigTypedef *config, const modSprigProtectionInputsTypedef *in) {
	modSprigProtectionResultTypedef result = {0, false, false, false};
	bool cellOverVoltage, packOverVoltage, overTemperature;
	bool hardUnderVoltage, softUnderVoltage, dischargeCold;

	// Over-voltage (bit 0): a cell or the pack total (D13). Blind to data it cannot trust (D10).
	cellOverVoltage = in->cellsKnown && in->cellVoltageHigh > config->cellHardOverVoltage;
	packOverVoltage = in->packKnown && in->packVoltage > config->packHardOverVoltage;
	if(modSprigProtectionPersists(&state->overVoltage, cellOverVoltage || packOverVoltage, in->nowMs, config->voltageTripDelayMs))
		state->tripped |= SPRIG_FAULT_A_CELL_OVER_VOLTAGE;

	// Discharge (bit 2) and charge (bit 3) over-current: threshold held for a delay (D8, D14).
	if(modSprigProtectionPersists(&state->dischargeOverCurrent, in->packKnown && -in->packCurrent > config->dischargeTripCurrent, in->nowMs, config->dischargeTripDelayMs))
		state->tripped |= SPRIG_FAULT_A_DISCHARGE_OVER_CURRENT;
	if(modSprigProtectionPersists(&state->chargeOverCurrent, in->packKnown && in->packCurrent > config->chargeTripCurrent, in->nowMs, config->chargeTripDelayMs))
		state->tripped |= SPRIG_FAULT_A_CHARGE_OVER_CURRENT;

	// Over-temperature (bit 4): cells or BMS board.
	overTemperature = in->tempBatteryHigh >= config->tempBatteryDischargeMax || (config->tempBMSEnabled && in->tempBMSHigh > config->tempBMSMax);
	if(modSprigProtectionPersists(&state->overTemperature, overTemperature, in->nowMs, config->temperatureTripDelayMs))
		state->tripped |= SPRIG_FAULT_A_OVER_TEMPERATURE;

	// Low side (D12): never opens in flight, blocks closing.
	hardUnderVoltage = in->cellsKnown && in->cellVoltageLow < config->cellHardUnderVoltage;
	softUnderVoltage = in->cellsKnown && in->cellVoltageLow <= config->cellSoftUnderVoltage;
	dischargeCold    = in->tempBatteryLow <= config->tempBatteryDischargeMin;

	result.faultsA = state->tripped;
	if(hardUnderVoltage)
		result.faultsA |= SPRIG_FAULT_A_CELL_UNDER_VOLTAGE;
	if(in->tempBatteryLow <= config->tempBatteryChargeMin) {
		result.faultsA      |= SPRIG_FAULT_A_CHARGE_UNDER_TEMP;
		result.chargeInhibit = true;
	}
	if(in->cellMonitorFault)
		result.faultsA |= SPRIG_FAULT_A_CELL_MONITOR_COMM;
	if(in->currentSensorFault)
		result.faultsA |= SPRIG_FAULT_A_CURRENT_SENSOR;

	result.trip         = state->tripped != 0;
	result.lowSideBlock = hardUnderVoltage || softUnderVoltage || dischargeCold;

	return result;
}
