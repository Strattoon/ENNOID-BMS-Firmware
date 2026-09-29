/*
	Sprig relay supervisor. See modSprigRelay.h.

	Sequence (spec "Sequence"):
	  STANDBY -> PRECHARGING when both request sources are 1, protections pass,
	  the DTI watch is healthy, HVIL is closed and nothing else blocks closing.
	  PRECHARGING -> ENERGIZED once the load voltage reaches
	  prechargeMatchFraction x pack voltage and, while the DTI watch is healthy,
	  the DTI 0x20 input voltage agrees within prechargeCrossCheckVolts.
	  PRECHARGING -> PRECHARGE_FAILED on prechargeTimeoutMs (latched).

	Hold policy (spec "Hold policy"): once ENERGIZED only the BMS protections or
	an agreed request withdrawal open the relays here. The power button and hard
	protection errors are handled by the operational state machine, which calls
	modSprigRelayForceOpen(). Every other problem only latches a faults_b bit.
 */

#include "modSprigRelay.h"
#include <math.h>

#define SPRIG_FAULT_B_PRECHARGE_BITS (SPRIG_FAULT_B_PRECHARGE_TIMEOUT | SPRIG_FAULT_B_PRECHARGE_MISMATCH)

static bool modSprigRelayElapsed(uint32_t nowMs, uint32_t startMs, uint32_t durationMs) {
	return (uint32_t)(nowMs - startMs) >= durationMs;
}

void modSprigRelayInit(modSprigRelayStateTypedef *state) {
	state->phase                 = SPRIG_PHASE_STANDBY;
	state->faultsB               = 0;
	state->lastOpenReason        = SPRIG_OPEN_NONE;
	state->prechargeStartMs      = 0;
	state->disagreementStartMs   = 0;
	state->disagreementTiming    = false;
	state->prechargeMismatchSeen = false;
}

void modSprigRelayForceOpen(modSprigRelayStateTypedef *state, sprigCanOpenReasonTypedef reason) {
	if(state->phase == SPRIG_PHASE_PRECHARGING || state->phase == SPRIG_PHASE_ENERGIZED)
		state->lastOpenReason = (uint8_t)reason;
	if(state->phase != SPRIG_PHASE_PRECHARGE_FAILED)
		state->phase = SPRIG_PHASE_STANDBY;
}

// Live faults_b conditions, before latching.
static uint8_t modSprigRelayConditions(modSprigRelayStateTypedef *state, const modSprigRelayConfigTypedef *config, const modSprigRelayInputsTypedef *in) {
	uint8_t conditions = 0;

	if(!in->dtiWatchHealthy)
		conditions |= SPRIG_FAULT_B_DTI_WATCH_TIMEOUT;
	else if(in->dtiFaultCode != 0)
		conditions |= SPRIG_FAULT_B_DTI_FAULT;

	if(!in->configValid)
		conditions |= SPRIG_FAULT_B_CONFIG_INVALID;

	if(!in->expanderOk)
		conditions |= SPRIG_FAULT_B_EXPANDER;
	else if(!in->hvilClosed)
		conditions |= SPRIG_FAULT_B_HVIL_OPEN;

	// A disagreement needs both sources known; it must persist for the debounce time.
	if(in->expanderOk && in->dtiWatchHealthy && (in->dtiCanRequest != in->hwRequest)) {
		if(!state->disagreementTiming) {
			state->disagreementTiming  = true;
			state->disagreementStartMs = in->nowMs;
		}
		if(modSprigRelayElapsed(in->nowMs, state->disagreementStartMs, config->relayRequestDebounceMs))
			conditions |= SPRIG_FAULT_B_REQUEST_DISAGREEMENT;
	}else{
		state->disagreementTiming = false;
	}

	return conditions;
}

static bool modSprigRelayBothRequested(const modSprigRelayInputsTypedef *in) {
	return in->dtiWatchHealthy && in->dtiCanRequest && in->expanderOk && in->hwRequest;
}

static bool modSprigRelayBothWithdrawn(const modSprigRelayInputsTypedef *in) {
	return in->dtiWatchHealthy && !in->dtiCanRequest && in->expanderOk && !in->hwRequest;
}

// Everything that stops a close, apart from the requests themselves.
static bool modSprigRelayCloseBlocked(uint8_t conditions, const modSprigRelayInputsTypedef *in) {
	const uint8_t blocking = SPRIG_FAULT_B_DTI_WATCH_TIMEOUT | SPRIG_FAULT_B_CONFIG_INVALID | SPRIG_FAULT_B_HVIL_OPEN |
	                         SPRIG_FAULT_B_REQUEST_DISAGREEMENT | SPRIG_FAULT_B_EXPANDER;
	return (conditions & blocking) || !in->protectionsOk || in->sensingFault || in->maintenance;
}

static bool modSprigRelayCrossCheckOk(const modSprigRelayConfigTypedef *config, const modSprigRelayInputsTypedef *in) {
	return fabsf(in->dtiInputVoltage - in->loadVoltage) <= config->prechargeCrossCheckVolts;
}

static void modSprigRelayOpen(modSprigRelayStateTypedef *state, modSprigRelayPhaseTypedef phase, sprigCanOpenReasonTypedef reason) {
	state->phase          = phase;
	state->lastOpenReason = (uint8_t)reason;
}

modSprigRelayOutputsTypedef modSprigRelayStep(modSprigRelayStateTypedef *state, const modSprigRelayConfigTypedef *config, const modSprigRelayInputsTypedef *in) {
	modSprigRelayOutputsTypedef out = {false, false, false};
	uint8_t conditions = modSprigRelayConditions(state, config, in);

	// Latched system faults clear only in STANDBY, once the condition is gone; do it
	// before a transition so a cleared bit does not ride along into PRECHARGING.
	if(state->phase == SPRIG_PHASE_STANDBY)
		state->faultsB = conditions;

	switch(state->phase) {
		case SPRIG_PHASE_STANDBY:
			if(modSprigRelayBothRequested(in) && !modSprigRelayCloseBlocked(conditions, in)) {
				state->phase                 = SPRIG_PHASE_PRECHARGING;
				state->prechargeStartMs      = in->nowMs;
				state->prechargeMismatchSeen = false;
			}
			break;

		case SPRIG_PHASE_PRECHARGING:
			if(!in->protectionsOk) {
				modSprigRelayOpen(state, SPRIG_PHASE_STANDBY, SPRIG_OPEN_PROTECTION);
			}else if(modSprigRelayBothWithdrawn(in)) {
				modSprigRelayOpen(state, SPRIG_PHASE_STANDBY, SPRIG_OPEN_REQUEST_WITHDRAWN);
			}else if(modSprigRelayCloseBlocked(conditions, in)) {
				// Not energized yet, so "will not close": abort the precharge.
				state->phase = SPRIG_PHASE_STANDBY;
				if(conditions & SPRIG_FAULT_B_DTI_WATCH_TIMEOUT)
					state->lastOpenReason = SPRIG_OPEN_DTI_WATCH_LOST;
			}else if(in->packVoltage > 0.0f && in->loadVoltage >= config->prechargeMatchFraction * in->packVoltage) {
				if(modSprigRelayCrossCheckOk(config, in))
					state->phase = SPRIG_PHASE_ENERGIZED;
				else
					state->prechargeMismatchSeen = true;
			}

			if(state->phase == SPRIG_PHASE_PRECHARGING && modSprigRelayElapsed(in->nowMs, state->prechargeStartMs, config->prechargeTimeoutMs)) {
				state->faultsB |= state->prechargeMismatchSeen ? SPRIG_FAULT_B_PRECHARGE_MISMATCH : SPRIG_FAULT_B_PRECHARGE_TIMEOUT;
				modSprigRelayOpen(state, SPRIG_PHASE_PRECHARGE_FAILED, SPRIG_OPEN_PRECHARGE_FAILURE);
			}
			break;

		case SPRIG_PHASE_ENERGIZED:
			if(!in->protectionsOk)
				modSprigRelayOpen(state, SPRIG_PHASE_STANDBY, SPRIG_OPEN_PROTECTION);
			else if(modSprigRelayBothWithdrawn(in))
				modSprigRelayOpen(state, SPRIG_PHASE_STANDBY, SPRIG_OPEN_REQUEST_WITHDRAWN);
			// Anything else holds; the conditions are latched below.
			break;

		case SPRIG_PHASE_PRECHARGE_FAILED:
		default:
			// Latched until the DTI agrees it no longer wants the bus.
			if(modSprigRelayBothWithdrawn(in))
				state->phase = SPRIG_PHASE_STANDBY;
			break;
	}

	if(state->phase == SPRIG_PHASE_STANDBY)
		state->faultsB = conditions;
	else
		state->faultsB |= conditions;

	switch(state->phase) {
		case SPRIG_PHASE_PRECHARGING:
			out.precharge = true;
			break;
		case SPRIG_PHASE_ENERGIZED:
			out.main      = true;
			out.dtiEnable = true;
			break;
		default:
			break;
	}

	return out;
}
