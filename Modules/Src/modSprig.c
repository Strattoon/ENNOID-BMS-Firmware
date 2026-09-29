/*
	Sprig BMS CAN protocol v1 on the ENNOID Gen 1 Master-HV. See modSprig.h.
 */

#include "modSprig.h"
#include "modSprigInputs.h"
#include "modSprigTerminal.h"
#include "libSprigCan.h"
#include "modCAN.h"
#include "modDelay.h"
#include "driverHWSwitches.h"
#include <math.h>

typedef struct {
	uint16_t id;
	uint16_t periodMs;
	uint16_t offsetMs;       // Spreads the frames so they do not queue in one burst
	uint32_t lastTick;
	uint8_t  counter;
} modSprigTxSlotTypedef;

static modSprigTxSlotTypedef modSprigTxSchedule[] = {
	{SPRIG_CAN_ID_STATE,   50,   0, 0, 0},
	{SPRIG_CAN_ID_PACK,    50,  10, 0, 0},
	{SPRIG_CAN_ID_LIMITS,  100, 20, 0, 0},
	{SPRIG_CAN_ID_CELLS,   200, 30, 0, 0},
	{SPRIG_CAN_ID_TEMPS,   500, 40, 0, 0},
	{SPRIG_CAN_ID_ENERGY, 1000, 45, 0, 0},
	{SPRIG_CAN_ID_IDENT,  1000, 55, 0, 0},
};
#define SPRIG_TX_SLOTS (sizeof(modSprigTxSchedule) / sizeof(modSprigTxSchedule[0]))

static modPowerElectronicsPackStateTypedef *modSprigPackState;
static modConfigGeneralConfigStructTypedef *modSprigConfig;
static modSprigRelayStateTypedef            modSprigRelayState;
static uint8_t                              modSprigFaultsA;
static bool                                 modSprigMaintenanceActive;

void modSprigInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer) {
	uint32_t now = HAL_GetTick();

	modSprigPackState         = packState;
	modSprigConfig            = generalConfigPointer;
	modSprigFaultsA           = 0;
	modSprigMaintenanceActive = false;
	modSprigRelayInit(&modSprigRelayState);
	modSprigInputsInit(generalConfigPointer);

	for(uint8_t slot = 0; slot < SPRIG_TX_SLOTS; slot++)
		modSprigTxSchedule[slot].lastTick = now + modSprigTxSchedule[slot].offsetMs - modSprigTxSchedule[slot].periodMs;

	modSprigTerminalInit(generalConfigPointer);
}

bool modSprigEnabled(void) {
	return modSprigConfig && modSprigConfig->sprigCanEnabled;
}

bool modSprigMaintenance(void) {
	return modSprigMaintenanceActive;
}

bool modSprigRelaysOpen(void) {
	return !driverHWSwitchesGetSwitchState(SWITCH_PRECHARGE) && !driverHWSwitchesGetSwitchState(SWITCH_DISCHARGE) &&
	       !driverHWSwitchesGetSwitchState(SWITCH_CHARGE) && !driverHWSwitchesGetSwitchState(SWITCH_DISCHARGEHV);
}

bool modSprigSetMaintenance(bool enable) {
	if(enable) {
		// Only with the relays open and waiting: MAINTENANCE opens the configuration channel.
		if(modSprigPackState->operationalState != OP_STATE_STANDBY || modSprigRelayState.phase != SPRIG_PHASE_STANDBY || !modSprigRelaysOpen())
			return false;
	}
	modSprigMaintenanceActive = enable;
	return true;
}

void modSprigCANReceive(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data) {
	if(modSprigEnabled())
		modSprigInputsDtiReceive(id, extended, dlc, data);
}

// ---- Protection and sensing faults ----

static uint8_t modSprigFaultsALive(void) {
	modPowerElectronicsPackStateTypedef *pack = modSprigPackState;
	modConfigGeneralConfigStructTypedef *cfg  = modSprigConfig;
	bool cellsKnown = !pack->cellMonitorCommFault && pack->cellVoltageHigh > 0.0f;         // 0 V: not read yet
	uint8_t faults = 0;

	if(cellsKnown && pack->cellVoltageHigh > cfg->cellHardOverVoltage)
		faults |= SPRIG_FAULT_A_CELL_OVER_VOLTAGE;
	if(cellsKnown && (pack->cellVoltageLow < cfg->cellHardUnderVoltage || pack->cellVoltageLow <= cfg->cellLCSoftUnderVoltage))
		faults |= SPRIG_FAULT_A_CELL_UNDER_VOLTAGE;
	if(pack->dischargeOverCurrentTrip)
		faults |= SPRIG_FAULT_A_DISCHARGE_OVER_CURRENT;
	if(pack->chargeOverCurrentTrip)
		faults |= SPRIG_FAULT_A_CHARGE_OVER_CURRENT;
	if(pack->tempBatteryHigh >= cfg->allowedTempBattDischargingMax || (cfg->tempEnableMaskBMS && pack->tempBMSHigh > cfg->allowedTempBMSMax))
		faults |= SPRIG_FAULT_A_OVER_TEMPERATURE;
	if(pack->tempBatteryLow <= cfg->allowedTempBattChargingMin)
		faults |= SPRIG_FAULT_A_CHARGE_UNDER_TEMP;
	if(pack->cellMonitorCommFault)
		faults |= SPRIG_FAULT_A_CELL_MONITOR_COMM;
	if(pack->currentSensorFault)
		faults |= SPRIG_FAULT_A_CURRENT_SENSOR;

	return faults;
}

static void modSprigUpdateFaults(void) {
	uint8_t live = modSprigFaultsALive();

	// Latched protection faults clear only in STANDBY, once the condition is gone.
	if(modSprigPackState->operationalState == OP_STATE_STANDBY)
		modSprigFaultsA = live;
	else
		modSprigFaultsA |= live;
}

// Permissions bits 0 and 2 (spec "Sequence" step 2) and no hard protection error.
static bool modSprigProtectionsOk(void) {
	modPowerElectronicsPackStateTypedef *pack = modSprigPackState;

	return pack->disChargeLCAllowed && pack->packInSOADischarge &&
	       pack->packOperationalCellState != PACK_STATE_ERROR_HARD_CELLVOLTAGE &&
	       pack->packOperationalCellState != PACK_STATE_ERROR_TEMPERATURE &&
	       pack->packOperationalCellState != PACK_STATE_ERROR_OVER_CURRENT &&
	       !pack->dischargeOverCurrentTrip && !pack->chargeOverCurrentTrip;
}

// ---- Relay supervisor ----

modSprigRelayPhaseTypedef modSprigRelayTask(void) {
	modSprigRelayConfigTypedef  relayConfig;
	modSprigRelayInputsTypedef  in;
	modSprigRelayOutputsTypedef out;
	modSprigDtiWatchTypedef     watch    = modSprigInputsDtiWatch();
	modSprigExpanderTypedef     expander = modSprigInputsExpander();

	modSprigUpdateFaults();

	relayConfig.relayRequestDebounceMs   = modSprigConfig->relayRequestDebounceMs;
	relayConfig.prechargeTimeoutMs       = modSprigConfig->prechargeTimeoutMs;
	relayConfig.prechargeMatchFraction   = modSprigConfig->prechargeMatchFraction;
	relayConfig.prechargeCrossCheckVolts = modSprigConfig->prechargeCrossCheckVolts;

	in.nowMs           = HAL_GetTick();
	in.configValid     = modConfigSprigValid(modSprigConfig);
	in.maintenance     = modSprigMaintenanceActive;
	in.protectionsOk   = modSprigProtectionsOk();
	in.sensingFault    = (modSprigFaultsA & (SPRIG_FAULT_A_CELL_MONITOR_COMM | SPRIG_FAULT_A_CURRENT_SENSOR)) != 0;
	in.dtiWatchHealthy = watch.healthy;
	in.dtiCanRequest   = watch.canRequest;
	in.dtiFaultCode    = watch.faultCode;
	in.dtiInputVoltage = watch.inputVoltage;
	in.expanderOk      = expander.ok;
	in.hwRequest       = expander.request;
	in.hvilClosed      = expander.hvilClosed;
	in.loadVoltage     = modSprigPackState->loCurrentLoadVoltage;
	in.packVoltage     = modSprigPackState->packVoltage;

	out = modSprigRelayStep(&modSprigRelayState, &relayConfig, &in);
	modPowerElectronicsSetSprigRelays(out.precharge, out.main, out.dtiEnable);

	return modSprigRelayState.phase;
}

void modSprigRelaysForcedOpen(sprigCanOpenReasonTypedef reason) {
	modSprigRelayForceOpen(&modSprigRelayState, reason);
}

// ---- BMS_STATE content ----

static uint8_t modSprigState(void) {
	if(modSprigMaintenanceActive)
		return SPRIG_STATE_MAINTENANCE;

	switch(modSprigPackState->operationalState) {
		case OP_STATE_INIT:            return SPRIG_STATE_INIT;
		case OP_STATE_STANDBY:
		case OP_STATE_EXTERNAL:
		case OP_STATE_CHARGED:         return SPRIG_STATE_STANDBY;
		case OP_STATE_PRE_CHARGE:      return SPRIG_STATE_PRECHARGING;
		case OP_STATE_LOAD_ENABLED:    return SPRIG_STATE_ENERGIZED;
		case OP_STATE_CHARGING:        return SPRIG_STATE_CHARGING;
		case OP_STATE_BALANCING:       return SPRIG_STATE_BALANCING;
		case OP_STATE_ERROR_PRECHARGE: return SPRIG_STATE_PRECHARGE_FAILED;
		case OP_STATE_BATTERY_DEAD:
		case OP_STATE_ERROR:           return SPRIG_STATE_FAULT;
		case OP_STATE_POWER_DOWN:      return SPRIG_STATE_POWER_DOWN;
		case OP_STATE_FORCEON:         return SPRIG_STATE_FORCED_ON;
		default:                       return SPRIG_STATE_FAULT;
	}
}

// relay_outputs reports the driven coil outputs, read back from the pins.
static uint8_t modSprigRelayOutputs(void) {
	uint8_t outputs = 0;

	if(driverHWSwitchesGetSwitchState(SWITCH_PRECHARGE))   outputs |= SPRIG_RELAY_PRECHARGE;
	if(driverHWSwitchesGetSwitchState(SWITCH_DISCHARGE))   outputs |= SPRIG_RELAY_DISCHARGE;
	if(driverHWSwitchesGetSwitchState(SWITCH_CHARGE))      outputs |= SPRIG_RELAY_CHARGE;
	if(driverHWSwitchesGetSwitchState(SWITCH_DISCHARGEHV)) outputs |= SPRIG_RELAY_DISCHARGE_NEGATIVE;

	return outputs;
}

static uint8_t modSprigPermissions(void) {
	modPowerElectronicsPackStateTypedef *pack = modSprigPackState;
	modSprigDtiWatchTypedef watch    = modSprigInputsDtiWatch();
	modSprigExpanderTypedef expander = modSprigInputsExpander();
	uint8_t permissions = 0;

	if(pack->disChargeLCAllowed)            permissions |= SPRIG_PERM_DISCHARGE_ALLOWED;
	if(pack->chargeAllowed)                 permissions |= SPRIG_PERM_CHARGE_ALLOWED;
	if(pack->packInSOADischarge)            permissions |= SPRIG_PERM_IN_SOA;
	if(watch.healthy)                       permissions |= SPRIG_PERM_DTI_WATCH_HEALTHY;
	if(watch.canRequest)                    permissions |= SPRIG_PERM_CAN_REQUEST;
	if(pack->powerButtonActuated)           permissions |= SPRIG_PERM_POWER_BUTTON;
	if(pack->balanceActive)                 permissions |= SPRIG_PERM_BALANCING;
	if(expander.ok && expander.request)     permissions |= SPRIG_PERM_HW_REQUEST;

	return permissions;
}

// ---- Frames ----

static void modSprigCellExtremes(libSprigCanCellsFrameTypedef *cells) {
	modPowerElectronicsPackStateTypedef *pack = modSprigPackState;
	uint16_t count = (uint16_t)modSprigConfig->noOfCellsSeries * modSprigConfig->noOfParallelModules;
	uint16_t minIndex = 0, maxIndex = 0;

	if(count > NoOfCellsPossibleOnBMS)
		count = NoOfCellsPossibleOnBMS;

	for(uint16_t i = 1; i < count; i++) {
		if(pack->cellVoltagesIndividual[i].cellVoltage < pack->cellVoltagesIndividual[minIndex].cellVoltage)
			minIndex = i;
		if(pack->cellVoltagesIndividual[i].cellVoltage > pack->cellVoltagesIndividual[maxIndex].cellVoltage)
			maxIndex = i;
	}

	cells->cellVoltageMinMilliVolt = (uint16_t)libSprigCanScale(pack->cellVoltageLow, 1000.0f, 0, 0xFFFF, 0);
	cells->cellVoltageMaxMilliVolt = (uint16_t)libSprigCanScale(pack->cellVoltageHigh, 1000.0f, 0, 0xFFFF, 0);
	cells->cellIndexMin = (count && minIndex < SPRIG_CAN_CELL_INDEX_UNKNOWN) ? (uint8_t)minIndex : SPRIG_CAN_CELL_INDEX_UNKNOWN;
	cells->cellIndexMax = (count && maxIndex < SPRIG_CAN_CELL_INDEX_UNKNOWN) ? (uint8_t)maxIndex : SPRIG_CAN_CELL_INDEX_UNKNOWN;
}

static uint8_t modSprigHardwareId(void) {
	switch(modSprigConfig->cellMonitorType) {
		case CELL_MON_LTC6811_1: return SPRIG_HARDWARE_ID_GEN1_LTC6811;
		case CELL_MON_LTC6813_1: return SPRIG_HARDWARE_ID_GEN1_LTC6813;
		default:                 return 0;                                       // Reserved
	}
}

static void modSprigBuildFrame(uint16_t id, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	modPowerElectronicsPackStateTypedef *pack = modSprigPackState;
	modConfigGeneralConfigStructTypedef *cfg  = modSprigConfig;

	switch(id) {
		case SPRIG_CAN_ID_STATE: {
			libSprigCanStateFrameTypedef s = {modSprigState(), modSprigRelayOutputs(), modSprigPermissions(),
			                                  modSprigFaultsA, modSprigRelayState.faultsB, modSprigRelayState.lastOpenReason};
			libSprigCanPackState(&s, counter, frame);
			} break;
		case SPRIG_CAN_ID_PACK: {
			libSprigCanPackFrameTypedef p;
			p.packVoltageDeciVolt = (uint16_t)libSprigCanScale(pack->packVoltage, 10.0f, 0, 0xFFFF, 0);
			p.packCurrentDeciAmp  = (int16_t)libSprigCanScale(-pack->packCurrent, 10.0f, -32768, 32767, 0);   // ENNOID: positive = charge
			p.busVoltageDeciVolt  = (uint16_t)libSprigCanScale(pack->loCurrentLoadVoltage, 10.0f, 0, 0xFFFE, SPRIG_CAN_BUS_VOLTAGE_UNAVAILABLE);
			libSprigCanPackPack(&p, counter, frame);
			} break;
		case SPRIG_CAN_ID_LIMITS: {
			libSprigCanLimitsFrameTypedef l;
			l.dischargeCurrentLimitDeciAmp = (uint16_t)libSprigCanScale(cfg->maxDischargeCurrent * pack->throttleDutyDischarge / 1000.0f, 10.0f, 0, 0xFFFF, 0);
			l.chargeCurrentLimitDeciAmp    = (uint16_t)libSprigCanScale(cfg->maxChargeCurrent * pack->throttleDutyCharge / 1000.0f, 10.0f, 0, 0xFFFF, 0);
			l.stateOfChargeCentiPercent    = (uint16_t)libSprigCanScale(pack->SoC, 100.0f, 0, 10000, 0);
			libSprigCanPackLimits(&l, counter, frame);
			} break;
		case SPRIG_CAN_ID_CELLS: {
			libSprigCanCellsFrameTypedef c;
			modSprigCellExtremes(&c);
			libSprigCanPackCells(&c, counter, frame);
			} break;
		case SPRIG_CAN_ID_TEMPS: {
			libSprigCanTempsFrameTypedef t;
			t.cellTempMaxDeciC = (int16_t)libSprigCanScale(pack->tempBatteryHigh, 10.0f, -32768, 32767, 0);
			t.cellTempAvgDeciC = (int16_t)libSprigCanScale(pack->tempBatteryAverage, 10.0f, -32768, 32767, 0);
			t.bmsTempMaxC      = (int8_t)libSprigCanScale(pack->tempBMSHigh, 1.0f, -128, 127, 0);
			libSprigCanPackTemps(&t, counter, frame);
			} break;
		case SPRIG_CAN_ID_ENERGY: {
			libSprigCanEnergyFrameTypedef e;
			e.remainingCapacityCentiAh = (uint16_t)libSprigCanScale(pack->SoCCapacityAh, 100.0f, 0, 0xFFFF, 0);
			e.fullCapacityCentiAh      = (uint16_t)libSprigCanScale(cfg->batteryCapacity, 100.0f, 0, 0xFFFF, 0);
			e.seriesCells              = cfg->noOfCellsSeries;
			libSprigCanPackEnergy(&e, counter, frame);
			} break;
		default: {
			libSprigCanIdentFrameTypedef i = {SPRIG_FIRMWARE_BUILD, cfg->configRevision, modSprigHardwareId()};
			libSprigCanPackIdent(&i, counter, frame);
			} break;
	}
}

static void modSprigTransmitSchedule(void) {
	uint32_t now = HAL_GetTick();
	uint8_t  frame[SPRIG_CAN_DLC];

	for(uint8_t slot = 0; slot < SPRIG_TX_SLOTS; slot++) {
		modSprigTxSlotTypedef *s = &modSprigTxSchedule[slot];

		if((uint32_t)(now - s->lastTick) < s->periodMs)
			continue;

		// Keep the cadence, but do not burst to catch up after a long main-loop stall.
		s->lastTick += s->periodMs;
		if((uint32_t)(now - s->lastTick) >= s->periodMs)
			s->lastTick = now;

		modSprigBuildFrame(s->id, s->counter, frame);
		s->counter = (s->counter + 1) & 0x0F;
		modCANTransmitStandardID(s->id, frame, SPRIG_CAN_DLC);
	}
}

void modSprigTask(void) {
	if(!modSprigEnabled())
		return;

	modSprigInputsExpanderTask();
	modSprigUpdateFaults();
	modSprigTransmitSchedule();
}
