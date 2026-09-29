/*
	Host unit test: Sprig relay supervisor (spec "Sequence" and "Hold policy").
 */

#include "sprig_test.h"
#include "modSprigRelay.h"

static const modSprigRelayConfigTypedef config = {
	.relayRequestDebounceMs   = 100,
	.prechargeTimeoutMs       = 2000,
	.prechargeMatchFraction   = 0.95f,
	.prechargeCrossCheckVolts = 15.0f,
};

static modSprigRelayInputsTypedef healthyRequested(uint32_t nowMs) {
	modSprigRelayInputsTypedef in = {
		.nowMs = nowMs, .configValid = true, .maintenance = false, .protectionsOk = true, .sensingFault = false,
		.dtiWatchHealthy = true, .dtiCanRequest = true, .dtiFaultCode = 0, .dtiInputVoltage = 0.0f,
		.expanderOk = true, .hwRequest = true, .hvilClosed = true, .loadVoltage = 0.0f, .packVoltage = 396.0f,
	};
	return in;
}

// Drive STANDBY -> PRECHARGING -> ENERGIZED; returns the time of the last step.
static uint32_t energize(modSprigRelayStateTypedef *state) {
	modSprigRelayInputsTypedef in = healthyRequested(0);
	modSprigRelayInit(state);
	modSprigRelayStep(state, &config, &in);
	in.nowMs = 500; in.loadVoltage = 390.0f; in.dtiInputVoltage = 388.0f;
	modSprigRelayStep(state, &config, &in);
	return in.nowMs;
}

static void testSequence(void) {
	modSprigRelayStateTypedef state;
	modSprigRelayInputsTypedef in = healthyRequested(0);
	modSprigRelayOutputsTypedef out;

	modSprigRelayInit(&state);
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("both requests -> PRECHARGING, precharge driven", state.phase == SPRIG_PHASE_PRECHARGING && out.precharge && !out.main && !out.dtiEnable);

	in.nowMs = 200; in.loadVoltage = 300.0f; in.dtiInputVoltage = 300.0f;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("below match fraction stays PRECHARGING", state.phase == SPRIG_PHASE_PRECHARGING && out.precharge);

	in.nowMs = 400; in.loadVoltage = 380.0f; in.dtiInputVoltage = 379.0f;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("match + cross-check -> ENERGIZED, main + DTI enable, precharge open",
		state.phase == SPRIG_PHASE_ENERGIZED && out.main && out.dtiEnable && !out.precharge && state.faultsB == 0);

	in.nowMs = 600; in.dtiCanRequest = false; in.hwRequest = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("agreed withdrawal -> STANDBY, reason 1, all open",
		state.phase == SPRIG_PHASE_STANDBY && state.lastOpenReason == SPRIG_OPEN_REQUEST_WITHDRAWN && !out.main && !out.precharge && !out.dtiEnable);
}

static void testWillNotClose(void) {
	modSprigRelayStateTypedef state;
	modSprigRelayInputsTypedef in;

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.hvilClosed = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("HVIL open: stays STANDBY with faults_b bit 5", state.phase == SPRIG_PHASE_STANDBY && state.faultsB == SPRIG_FAULT_B_HVIL_OPEN);

	in.hvilClosed = true; in.nowMs = 10;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("HVIL closes again: fault clears in STANDBY and it closes", state.phase == SPRIG_PHASE_PRECHARGING && state.faultsB == 0);

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.dtiWatchHealthy = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("DTI watch lost: will not close, faults_b bit 2", state.phase == SPRIG_PHASE_STANDBY && (state.faultsB & SPRIG_FAULT_B_DTI_WATCH_TIMEOUT));

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.expanderOk = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("expander fault: will not close, faults_b bit 7 only", state.phase == SPRIG_PHASE_STANDBY && state.faultsB == SPRIG_FAULT_B_EXPANDER);

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.configValid = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("config invalid: will not close, faults_b bit 4", state.phase == SPRIG_PHASE_STANDBY && state.faultsB == SPRIG_FAULT_B_CONFIG_INVALID);

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.sensingFault = true;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("sensing fault: will not close (D10)", state.phase == SPRIG_PHASE_STANDBY);

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.protectionsOk = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("protections fail: will not close", state.phase == SPRIG_PHASE_STANDBY);

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.maintenance = true;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("maintenance: will not close", state.phase == SPRIG_PHASE_STANDBY);

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.hwRequest = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("CAN request only: will not close, no fault before debounce", state.phase == SPRIG_PHASE_STANDBY && state.faultsB == 0);
	in.nowMs = 100;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("disagreement past debounce: faults_b bit 6", state.phase == SPRIG_PHASE_STANDBY && state.faultsB == SPRIG_FAULT_B_REQUEST_DISAGREEMENT);

	modSprigRelayInit(&state);
	in = healthyRequested(0); in.dtiFaultCode = 4;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("DTI fault code: faults_b bit 3 but does not block (spec step 2)", state.phase == SPRIG_PHASE_PRECHARGING && (state.faultsB & SPRIG_FAULT_B_DTI_FAULT));
}

static void testHoldPolicy(void) {
	modSprigRelayStateTypedef state;
	modSprigRelayInputsTypedef in;
	modSprigRelayOutputsTypedef out;
	uint32_t now;

	now = energize(&state);
	in = healthyRequested(now + 50); in.loadVoltage = 390.0f; in.dtiWatchHealthy = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("energized, DTI watch lost: hold, latch bit 2", state.phase == SPRIG_PHASE_ENERGIZED && out.main && out.dtiEnable && (state.faultsB & SPRIG_FAULT_B_DTI_WATCH_TIMEOUT));
	in.nowMs += 50; in.hwRequest = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("energized, watch lost and hardwired drops: hold (drop unconfirmed)", state.phase == SPRIG_PHASE_ENERGIZED && out.main);
	in.nowMs += 50; in.dtiWatchHealthy = true; in.hwRequest = true;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("watch recovers while energized: bit 2 stays latched", (state.faultsB & SPRIG_FAULT_B_DTI_WATCH_TIMEOUT) != 0);

	now = energize(&state);
	in = healthyRequested(now); in.loadVoltage = 390.0f; in.hwRequest = false;
	for(uint32_t t = 0; t <= 200; t += 50) {
		in.nowMs = now + t;
		out = modSprigRelayStep(&state, &config, &in);
	}
	SPRIG_CHECK("energized, requests disagree: hold, latch bit 6", state.phase == SPRIG_PHASE_ENERGIZED && out.main && state.faultsB == SPRIG_FAULT_B_REQUEST_DISAGREEMENT);

	now = energize(&state);
	in = healthyRequested(now + 50); in.loadVoltage = 390.0f; in.hvilClosed = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("energized, HVIL open: hold, latch bit 5", state.phase == SPRIG_PHASE_ENERGIZED && out.main && (state.faultsB & SPRIG_FAULT_B_HVIL_OPEN));

	now = energize(&state);
	in = healthyRequested(now + 50); in.loadVoltage = 390.0f; in.expanderOk = false; in.hwRequest = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("energized, expander fault: hold, latch bit 7", state.phase == SPRIG_PHASE_ENERGIZED && out.main && (state.faultsB & SPRIG_FAULT_B_EXPANDER));

	now = energize(&state);
	in = healthyRequested(now + 50); in.loadVoltage = 390.0f; in.sensingFault = true;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("energized, sensing fault: hold (D10)", state.phase == SPRIG_PHASE_ENERGIZED && out.main);

	now = energize(&state);
	in = healthyRequested(now + 50); in.loadVoltage = 390.0f; in.configValid = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("energized, config invalid: hold, latch bit 4", state.phase == SPRIG_PHASE_ENERGIZED && out.main && (state.faultsB & SPRIG_FAULT_B_CONFIG_INVALID));

	now = energize(&state);
	in = healthyRequested(now + 50); in.loadVoltage = 390.0f; in.protectionsOk = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("energized, protection: open, reason 2", state.phase == SPRIG_PHASE_STANDBY && !out.main && !out.dtiEnable && state.lastOpenReason == SPRIG_OPEN_PROTECTION);

	now = energize(&state);
	modSprigRelayForceOpen(&state, SPRIG_OPEN_POWER_BUTTON);
	SPRIG_CHECK("power button force-open: reason 5", state.phase == SPRIG_PHASE_STANDBY && state.lastOpenReason == SPRIG_OPEN_POWER_BUTTON);
}

static void testPrechargeFailure(void) {
	modSprigRelayStateTypedef state;
	modSprigRelayInputsTypedef in;
	modSprigRelayOutputsTypedef out;

	modSprigRelayInit(&state);
	in = healthyRequested(0);
	modSprigRelayStep(&state, &config, &in);
	in.nowMs = 1999; in.loadVoltage = 200.0f;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("before timeout still PRECHARGING", state.phase == SPRIG_PHASE_PRECHARGING);
	in.nowMs = 2000;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("timeout -> PRECHARGE_FAILED, bit 0, reason 4, all open",
		state.phase == SPRIG_PHASE_PRECHARGE_FAILED && state.faultsB == SPRIG_FAULT_B_PRECHARGE_TIMEOUT &&
		state.lastOpenReason == SPRIG_OPEN_PRECHARGE_FAILURE && !out.precharge && !out.main);
	in.nowMs = 3000; in.loadVoltage = 0.0f;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("PRECHARGE_FAILED is latched while the request stays", state.phase == SPRIG_PHASE_PRECHARGE_FAILED);
	in.nowMs = 3100; in.dtiCanRequest = false; in.hwRequest = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("agreed withdrawal releases the latch, bits clear in STANDBY", state.phase == SPRIG_PHASE_STANDBY && state.faultsB == 0);

	modSprigRelayInit(&state);
	in = healthyRequested(0);
	modSprigRelayStep(&state, &config, &in);
	in.nowMs = 500; in.loadVoltage = 390.0f; in.dtiInputVoltage = 300.0f;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("DTI disagrees with load voltage: does not close main", state.phase == SPRIG_PHASE_PRECHARGING && !out.main);
	in.nowMs = 2000;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("mismatch until timeout -> PRECHARGE_FAILED with bit 1", state.phase == SPRIG_PHASE_PRECHARGE_FAILED && state.faultsB == SPRIG_FAULT_B_PRECHARGE_MISMATCH);

	modSprigRelayInit(&state);
	in = healthyRequested(0);
	modSprigRelayStep(&state, &config, &in);
	in.nowMs = 100; in.dtiWatchHealthy = false;
	out = modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("watch lost during precharge: abort to STANDBY, reason 3", state.phase == SPRIG_PHASE_STANDBY && !out.precharge && state.lastOpenReason == SPRIG_OPEN_DTI_WATCH_LOST);

	modSprigRelayInit(&state);
	in = healthyRequested(0);
	modSprigRelayStep(&state, &config, &in);
	in.nowMs = 100; in.protectionsOk = false;
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("protection during precharge: STANDBY, reason 2", state.phase == SPRIG_PHASE_STANDBY && state.lastOpenReason == SPRIG_OPEN_PROTECTION);
}

static void testTickWrap(void) {
	modSprigRelayStateTypedef state;
	modSprigRelayInputsTypedef in = healthyRequested(0xFFFFFF00u);

	modSprigRelayInit(&state);
	modSprigRelayStep(&state, &config, &in);
	in.nowMs = 0x00000100u; in.loadVoltage = 100.0f;    // 512 ms later, across the 32-bit wrap
	modSprigRelayStep(&state, &config, &in);
	SPRIG_CHECK("precharge timer survives HAL tick wrap", state.phase == SPRIG_PHASE_PRECHARGING);
}

int main(void) {
	testSequence();
	testWillNotClose();
	testHoldPolicy();
	testPrechargeFailure();
	testTickWrap();
	return sprigTestSummary("test_sprig_relay");
}
