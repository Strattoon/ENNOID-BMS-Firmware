/*
	Sprig relay-path inputs. See modSprigInputs.h.
 */

#include "modSprigInputs.h"
#include "libSprigCan.h"
#include "driverSWPCAL6416.h"
#include "modDelay.h"
#include "stm32f3xx_hal.h"

#define SPRIG_EXPANDER_POLL_MS            20
#define SPRIG_EXPANDER_FAIL_LIMIT         3     // Consecutive bad polls before faults_b bit 7 (60 ms)
#define SPRIG_EXPANDER_CONFIG_CHECK_MS    1000  // Re-verify the pull configuration, e.g. after an expander brown-out
#define SPRIG_EXPANDER_RECOVER_MS         1000  // Bus recovery interval while the expander keeps failing
#define SPRIG_EXPANDER_READBACK_LOW_BIT   6     // P0.6 strapped low
#define SPRIG_EXPANDER_READBACK_HIGH_BIT  7     // P0.7 strapped high

typedef struct {
	volatile uint32_t lastGeneral1Tick;
	volatile uint32_t lastGeneral3Tick;
	volatile uint32_t lastGeneral5Tick;
	volatile bool     seenGeneral1;
	volatile bool     seenGeneral3;
	volatile bool     seenGeneral5;
	volatile int16_t  inputVoltage;
	volatile uint8_t  faultCode;
	volatile bool     digitalOutput;
} modSprigDtiRxTypedef;

static modConfigGeneralConfigStructTypedef *modSprigInputsConfig;
static modSprigDtiRxTypedef                 modSprigDtiRx;

static bool     modSprigExpanderConfigured;
static uint8_t  modSprigExpanderFailCount;
static uint32_t modSprigExpanderPollLastTick;
static uint32_t modSprigExpanderConfigCheckLastTick;
static uint32_t modSprigExpanderRecoverLastTick;
static modSprigExpanderTypedef modSprigExpanderState;

void modSprigInputsInit(modConfigGeneralConfigStructTypedef *generalConfigPointer) {
	modSprigInputsConfig = generalConfigPointer;

	modSprigDtiRx.seenGeneral1 = false;
	modSprigDtiRx.seenGeneral3 = false;
	modSprigDtiRx.seenGeneral5 = false;

	modSprigExpanderConfigured          = false;
	modSprigExpanderFailCount           = SPRIG_EXPANDER_FAIL_LIMIT;               // Unknown until the first good read
	modSprigExpanderState.ok            = false;
	modSprigExpanderState.request       = false;
	modSprigExpanderState.hvilClosed    = false;
	modSprigExpanderPollLastTick        = HAL_GetTick();
	modSprigExpanderConfigCheckLastTick = HAL_GetTick();
	modSprigExpanderRecoverLastTick     = HAL_GetTick();
}

// ---- DTI watch ----

void modSprigInputsDtiReceive(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data) {
	libSprigDtiFrameTypedef frame;
	uint32_t now = HAL_GetTick();

	if(libSprigDtiDecode(modSprigInputsConfig->dtiIdFormat, modSprigInputsConfig->dtiNodeId, modSprigInputsConfig->dtiRelayRequestOutput,
	                     id, extended, dlc, data, &frame) != SPRIG_DTI_DECODED)
		return;                                                                      // Wrong node/format, or not decodable: it stays stale

	switch(frame.packet) {
		case SPRIG_DTI_PACKET_GENERAL_1:
			modSprigDtiRx.inputVoltage     = frame.inputVoltageVolt;
			modSprigDtiRx.lastGeneral1Tick = now;
			modSprigDtiRx.seenGeneral1     = true;
			break;
		case SPRIG_DTI_PACKET_GENERAL_3:
			modSprigDtiRx.faultCode        = frame.faultCode;
			modSprigDtiRx.lastGeneral3Tick = now;
			modSprigDtiRx.seenGeneral3     = true;
			break;
		default:
			modSprigDtiRx.digitalOutput    = frame.digitalOutput;
			modSprigDtiRx.lastGeneral5Tick = now;
			modSprigDtiRx.seenGeneral5     = true;
			break;
	}
}

static bool modSprigInputsFresh(bool seen, uint32_t lastTick, uint32_t now, uint32_t timeoutMs) {
	return seen && (uint32_t)(now - lastTick) < timeoutMs;
}

modSprigDtiWatchTypedef modSprigInputsDtiWatch(void) {
	modSprigDtiWatchTypedef watch;
	modSprigDtiRxTypedef    snapshot;
	uint32_t now;

	__disable_irq();                                                               // Consistent copy of the ISR-written fields
	snapshot.lastGeneral1Tick = modSprigDtiRx.lastGeneral1Tick;
	snapshot.lastGeneral3Tick = modSprigDtiRx.lastGeneral3Tick;
	snapshot.lastGeneral5Tick = modSprigDtiRx.lastGeneral5Tick;
	snapshot.seenGeneral1     = modSprigDtiRx.seenGeneral1;
	snapshot.seenGeneral3     = modSprigDtiRx.seenGeneral3;
	snapshot.seenGeneral5     = modSprigDtiRx.seenGeneral5;
	snapshot.inputVoltage     = modSprigDtiRx.inputVoltage;
	snapshot.faultCode        = modSprigDtiRx.faultCode;
	snapshot.digitalOutput    = modSprigDtiRx.digitalOutput;
	now = HAL_GetTick();
	__enable_irq();

	watch.healthy = modSprigInputsFresh(snapshot.seenGeneral1, snapshot.lastGeneral1Tick, now, modSprigInputsConfig->dtiWatchTimeoutMs) &&
	                modSprigInputsFresh(snapshot.seenGeneral3, snapshot.lastGeneral3Tick, now, modSprigInputsConfig->dtiWatchTimeoutMs) &&
	                modSprigInputsFresh(snapshot.seenGeneral5, snapshot.lastGeneral5Tick, now, modSprigInputsConfig->dtiWatchTimeoutMs);
	watch.canRequest   = watch.healthy && snapshot.digitalOutput;
	watch.faultCode    = snapshot.faultCode;
	watch.inputVoltage = (float)snapshot.inputVoltage;

	return watch;
}

// ---- Input expander ----

static uint16_t modSprigExpanderChannelMask(uint8_t channel) {
	return (uint16_t)(1u << (channel & 0x0F));
}

// Pull every used pin away from its "good" level, so a missing strap or opto reads as a fault or as inactive.
static void modSprigExpanderPulls(uint16_t *enable, uint16_t *pullUp) {
	uint16_t inputs = modSprigExpanderChannelMask(modSprigInputsConfig->relayRequestInput) | modSprigExpanderChannelMask(modSprigInputsConfig->hvilInput);

	*enable = inputs | (1u << SPRIG_EXPANDER_READBACK_LOW_BIT) | (1u << SPRIG_EXPANDER_READBACK_HIGH_BIT);
	*pullUp = (1u << SPRIG_EXPANDER_READBACK_LOW_BIT);                             // The strap to ground must win against a pull-up
#if SPRIG_EXPANDER_INPUT_ACTIVE_LOW
	*pullUp |= inputs;                                                             // Inactive (request off, HVIL open) when nothing drives the pin
#endif
}

static bool modSprigExpanderConfigure(void) {
	uint16_t enable, pullUp;

	modSprigExpanderPulls(&enable, &pullUp);
	return driverSWPCAL6416Init(modSprigInputsConfig->inputExpanderAddress, 0xFF, 0xFF,             // All pins inputs
	                            (uint8_t)enable, (uint8_t)(enable >> 8), (uint8_t)pullUp, (uint8_t)(pullUp >> 8));
}

static bool modSprigExpanderConfigIntact(void) {
	uint16_t enable, pullUp, readEnable, readPullUp;

	modSprigExpanderPulls(&enable, &pullUp);
	if(!driverSWPCAL6416ReadConfig(modSprigInputsConfig->inputExpanderAddress, PCAL6416_REG_PULL_ENABLE_PORT0, &readEnable))
		return false;
	if(!driverSWPCAL6416ReadConfig(modSprigInputsConfig->inputExpanderAddress, PCAL6416_REG_PULL_SELECT_PORT0, &readPullUp))
		return false;
	return readEnable == enable && (readPullUp & enable) == (pullUp & enable);
}

static bool modSprigExpanderActive(uint16_t inputs, uint8_t channel) {
	bool high = (inputs & modSprigExpanderChannelMask(channel)) != 0;
#if SPRIG_EXPANDER_INPUT_ACTIVE_LOW
	return !high;
#else
	return high;
#endif
}

static void modSprigExpanderFailed(void) {
	if(modSprigExpanderFailCount < SPRIG_EXPANDER_FAIL_LIMIT)
		modSprigExpanderFailCount++;

	// Last good values are kept until the limit; after that request and HVIL are unknown.
	if(modSprigExpanderFailCount >= SPRIG_EXPANDER_FAIL_LIMIT) {
		modSprigExpanderState.ok = false;
		if(modDelayTick1ms(&modSprigExpanderRecoverLastTick, SPRIG_EXPANDER_RECOVER_MS)) {
			driverHWI2C1Recover();                                                   // A slave holding SDA also blocks the OLED
			modSprigExpanderConfigured = false;
		}
	}
}

void modSprigInputsExpanderTask(void) {
	uint16_t inputs;

	if(!modDelayTick1ms(&modSprigExpanderPollLastTick, SPRIG_EXPANDER_POLL_MS))
		return;

	if(!modSprigExpanderConfigured) {
		modSprigExpanderConfigured = modSprigExpanderConfigure();
		if(!modSprigExpanderConfigured) {
			modSprigExpanderFailed();
			return;
		}
		modSprigExpanderConfigCheckLastTick = HAL_GetTick();
	}

	if(modDelayTick1ms(&modSprigExpanderConfigCheckLastTick, SPRIG_EXPANDER_CONFIG_CHECK_MS) && !modSprigExpanderConfigIntact()) {
		modSprigExpanderConfigured = false;
		modSprigExpanderFailed();
		return;
	}

	if(!driverSWPCAL6416ReadInputs(modSprigInputsConfig->inputExpanderAddress, &inputs)) {
		modSprigExpanderFailed();
		return;
	}

	if((inputs & (1u << SPRIG_EXPANDER_READBACK_LOW_BIT)) || !(inputs & (1u << SPRIG_EXPANDER_READBACK_HIGH_BIT))) {
		modSprigExpanderFailed();                                                    // Wrong device or board: reads back wrongly
		return;
	}

	modSprigExpanderFailCount        = 0;
	modSprigExpanderState.ok         = true;
	modSprigExpanderState.request    = modSprigExpanderActive(inputs, modSprigInputsConfig->relayRequestInput);
	modSprigExpanderState.hvilClosed = modSprigExpanderActive(inputs, modSprigInputsConfig->hvilInput);
}

modSprigExpanderTypedef modSprigInputsExpander(void) {
	return modSprigExpanderState;
}
