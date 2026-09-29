/*
	Host unit test: Sprig BMS protections (spec "Hold policy", D8, D10-D14) and the first-boot
	open-circuit-voltage SoC estimate.
 */

#include "sprig_test.h"
#include "modSprigProtection.h"
#include "libSprigOcv.h"

// Configuration defaults: 9 x MaxAmps 5200 11S in series (99S), 400 V pack limit.
static const modSprigProtectionConfigTypedef config = {
	.cellHardOverVoltage     = 4.20f,
	.packHardOverVoltage     = 400.0f,
	.cellHardUnderVoltage    = 3.00f,
	.cellSoftUnderVoltage    = 3.50f,
	.voltageTripDelayMs      = 600,
	.dischargeTripCurrent    = 234.0f,
	.dischargeTripDelayMs    = 1000,
	.chargeTripCurrent       = 26.0f,
	.chargeTripDelayMs       = 1000,
	.tempBatteryDischargeMax = 60.0f,                                                  // MaxAmps operating limit
	.tempBatteryDischargeMin = 0.0f,
	.tempBatteryChargeMin    = 0.0f,
	.tempBMSMax              = 80.0f,
	.tempBMSEnabled          = true,
	.temperatureTripDelayMs  = 600,
};

static modSprigProtectionInputsTypedef nominal(uint32_t nowMs) {
	modSprigProtectionInputsTypedef in = {
		.nowMs = nowMs, .cellsKnown = true, .cellVoltageHigh = 4.01f, .cellVoltageLow = 3.99f,
		.packKnown = true, .packVoltage = 396.0f, .packCurrent = 0.0f,
		.tempBatteryHigh = 25.0f, .tempBatteryLow = 22.0f, .tempBMSHigh = 30.0f,
		.cellMonitorFault = false, .currentSensorFault = false,
	};
	return in;
}

static void testNominal(void) {
	modSprigProtectionStateTypedef state;
	modSprigProtectionInputsTypedef in = nominal(0);
	modSprigProtectionResultTypedef r;

	modSprigProtectionInit(&state);
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("nominal pack: no faults, no trip, may close", r.faultsA == 0 && !r.trip && !r.lowSideBlock && !r.chargeInhibit);
}

static void testChargeTripDelay(void) {
	modSprigProtectionStateTypedef state;
	modSprigProtectionInputsTypedef in;
	modSprigProtectionResultTypedef r;

	modSprigProtectionInit(&state);
	in = nominal(0); in.packCurrent = 10.0f;                                       // Above maxChargeCurrent (5.2 A), below the trip
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("10 A charge (above the 5.2 A limit, below the 26 A trip): no trip (D14)", !r.trip);

	in.packCurrent = 40.0f;
	for(uint32_t t = 0; t <= 900; t += 100) {
		in.nowMs = 1000 + t;
		r = modSprigProtectionEvaluate(&state, &config, &in);
	}
	SPRIG_CHECK("40 A charge for 0.9 s: transient, holds", !r.trip && !(r.faultsA & SPRIG_FAULT_A_CHARGE_OVER_CURRENT));
	in.nowMs = 1950; in.packCurrent = 20.0f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 2000; in.packCurrent = 40.0f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("a dip below the threshold restarts the delay", !r.trip);
	in.nowMs = 3000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("40 A charge held 1 s: trips, faults_a bit 3", r.trip && (r.faultsA & SPRIG_FAULT_A_CHARGE_OVER_CURRENT));
	in.nowMs = 4000; in.packCurrent = 0.0f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("charge trip stays latched after the current stops (D11)", r.trip && (r.faultsA & SPRIG_FAULT_A_CHARGE_OVER_CURRENT));
}

static void testDischargeTripDelay(void) {
	modSprigProtectionStateTypedef state;
	modSprigProtectionInputsTypedef in;
	modSprigProtectionResultTypedef r;

	modSprigProtectionInit(&state);
	in = nominal(0); in.packCurrent = -300.0f;                                     // Discharge is negative in ENNOID's convention
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 999;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("300 A discharge for 0.999 s: holds", !r.trip);
	in.nowMs = 1000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("300 A discharge held 1 s: trips, faults_a bit 2", r.trip && r.faultsA == SPRIG_FAULT_A_DISCHARGE_OVER_CURRENT);

	modSprigProtectionInit(&state);
	in = nominal(0); in.packCurrent = -300.0f; in.packKnown = false; in.currentSensorFault = true;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 5000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("current-sensor fault: over-current blind, no trip, bit 7 (D10)", !r.trip && r.faultsA == SPRIG_FAULT_A_CURRENT_SENSOR);
}

static void testOverVoltage(void) {
	modSprigProtectionStateTypedef state;
	modSprigProtectionInputsTypedef in;
	modSprigProtectionResultTypedef r;

	modSprigProtectionInit(&state);
	in = nominal(0); in.packVoltage = 401.0f;                                      // Cells still below 4.20 V
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("pack 401 V: not yet (delay)", !r.trip);
	in.nowMs = 600;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("pack 401 V held: pack-level OV trips, faults_a bit 0 (D13)", r.trip && r.faultsA == SPRIG_FAULT_A_CELL_OVER_VOLTAGE);

	modSprigProtectionInit(&state);
	in = nominal(0); in.packVoltage = 399.9f;
	in.nowMs = 10000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("pack 399.9 V: no trip", !r.trip);

	modSprigProtectionInit(&state);
	in = nominal(0); in.cellVoltageHigh = 4.21f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 600;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("one cell at 4.21 V (above MaxAmps max) held: cell OV trips, bit 0", r.trip && (r.faultsA & SPRIG_FAULT_A_CELL_OVER_VOLTAGE));

	modSprigProtectionInit(&state);
	in = nominal(0); in.cellVoltageHigh = 4.15f;                                    // Imbalanced cell above the 4.00 V target
	in.nowMs = 10000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("cell at 4.15 V (imbalance margin): no trip", !r.trip);

	modSprigProtectionInit(&state);
	in = nominal(0); in.packVoltage = 450.0f; in.packKnown = false; in.currentSensorFault = true;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 5000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("pack sensor faulted: pack OV blind (D10)", !r.trip);

	modSprigProtectionInit(&state);
	in = nominal(0); in.cellVoltageHigh = 5.0f; in.cellsKnown = false; in.cellMonitorFault = true;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 5000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("cell monitor faulted: cell OV blind, bit 6 (D10)", !r.trip && r.faultsA == SPRIG_FAULT_A_CELL_MONITOR_COMM);
}

static void testLowSideHolds(void) {
	modSprigProtectionStateTypedef state;
	modSprigProtectionInputsTypedef in;
	modSprigProtectionResultTypedef r;

	modSprigProtectionInit(&state);
	in = nominal(0); in.cellVoltageLow = 3.45f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 60000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("soft UV (3.45 V) for a minute: no trip, no faults_a bit, blocks closing (D12)", !r.trip && r.faultsA == 0 && r.lowSideBlock);

	in.cellVoltageLow = 2.90f;
	in.nowMs = 120000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("hard UV (2.90 V): bit 1 set, never trips, blocks closing (D12)", !r.trip && r.faultsA == SPRIG_FAULT_A_CELL_UNDER_VOLTAGE && r.lowSideBlock);

	modSprigProtectionInit(&state);
	in = nominal(0); in.tempBatteryLow = -5.0f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 60000;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("cold pack (-5 C): no trip; warns bit 5, charge limit 0, blocks closing",
		!r.trip && r.faultsA == SPRIG_FAULT_A_CHARGE_UNDER_TEMP && r.chargeInhibit && r.lowSideBlock);
}

static void testOverTemperature(void) {
	modSprigProtectionStateTypedef state;
	modSprigProtectionInputsTypedef in;
	modSprigProtectionResultTypedef r;

	modSprigProtectionInit(&state);
	in = nominal(0); in.tempBatteryHigh = 61.0f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("cells 61 C (MaxAmps max 60 C): not yet (delay)", !r.trip);
	in.nowMs = 600;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("cells 61 C held: trips, bit 4", r.trip && r.faultsA == SPRIG_FAULT_A_OVER_TEMPERATURE);

	modSprigProtectionInit(&state);
	in = nominal(0); in.tempBMSHigh = 85.0f;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	in.nowMs = 600;
	r = modSprigProtectionEvaluate(&state, &config, &in);
	SPRIG_CHECK("BMS board 85 C held: trips, bit 4", r.trip && r.faultsA == SPRIG_FAULT_A_OVER_TEMPERATURE);
}

static void testOcv(void) {
	SPRIG_CHECK("OCV 4.20 V -> 100 %", fabsf(libSprigOcvStateOfCharge(4.20f) - 100.0f) < 0.01f);
	SPRIG_CHECK("OCV 3.84 V -> 50 %", fabsf(libSprigOcvStateOfCharge(3.84f) - 50.0f) < 0.01f);
	SPRIG_CHECK("OCV 3.00 V (MaxAmps minimum) -> 0 %", libSprigOcvStateOfCharge(3.00f) == 0.0f);
	SPRIG_CHECK("OCV 2.90 V -> 0 %", libSprigOcvStateOfCharge(2.90f) == 0.0f);
	SPRIG_CHECK("OCV NaN -> 0 %", libSprigOcvStateOfCharge(NAN) == 0.0f);
	float target = libSprigOcvStateOfCharge(4.00f);
	printf("  note: 4.00 V/cell resting (99S charge target) reads %.1f %% (MaxAmps end points, generic LiPo shape)\n", (double)target);
	SPRIG_CHECK("OCV 4.00 V (99S charge target) reads between 75 % and 85 %", target > 75.0f && target < 85.0f);
}

int main(void) {
	testNominal();
	testChargeTripDelay();
	testDischargeTripDelay();
	testOverVoltage();
	testLowSideHolds();
	testOverTemperature();
	testOcv();
	return sprigTestSummary("test_sprig_protection");
}
