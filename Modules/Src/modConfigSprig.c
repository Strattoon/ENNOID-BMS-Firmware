/*
	Sprig BMS CAN protocol v1 configuration: defaults, flight-build forcing and
	validation (Sprig-Flight-Director docs/architecture/sprig-bms-can-v1.md,
	"Required ENNOID firmware changes" and D8).
 */

#include "modConfig.h"
#include "libSprigCan.h"

#define SPRIG_EXPANDER_CHANNELS          16
#define SPRIG_EXPANDER_READBACK_LOW      6   // P0.6 strapped low on the expander board
#define SPRIG_EXPANDER_READBACK_HIGH     7   // P0.7 strapped high
#define SPRIG_MAX_DEBOUNCE_MS            10000

void modConfigLoadSprigDefaults(modConfigGeneralConfigStructTypedef *configLocation) {
	configLocation->sprigCanEnabled          = SPRIG_FLIGHT_BUILD;
	configLocation->dtiIdFormat              = SPRIG_DTI_FORMAT_UNSET;   // Required, no default
	configLocation->dtiNodeId                = SPRIG_DTI_NODE_UNSET;     // Required, no default
	configLocation->dtiRelayRequestOutput    = 1;                        // H1000: DTI digital output 1
	configLocation->inputExpanderAddress     = 0x20;
	configLocation->relayRequestInput        = 0;                        // P0.0
	configLocation->hvilInput                = 1;                        // P0.1
	configLocation->dtiEnableOutput          = dtiEnableOutputCooling;
	configLocation->canOpenChargerEnabled    = false;
	configLocation->configRevision           = 0;
	configLocation->relayRequestDebounceMs   = 100;
	configLocation->dtiWatchTimeoutMs        = 250;
	configLocation->prechargeTimeoutMs       = 1500;                     // Upstream timeoutLCPreCharge
	configLocation->prechargeMatchFraction   = 0.95f;
	configLocation->prechargeCrossCheckVolts = 20.0f;
	// Bench-safe defaults (D8), tuned on the bench before flight: one 11S 5200 mAh pack, string 1P.
	configLocation->maxDischargeCurrent      = 234.0f;                   // Continuous rating of one pack
	configLocation->maxChargeCurrent         = 5.2f;                     // 1 C
	configLocation->dischargeTripCurrent     = 234.0f;
	configLocation->dischargeTripDelayMs     = 1000;
	configLocation->chargeTripCurrent        = 26.0f;                    // The pack's 5 C maximum charge (D14)
	configLocation->chargeTripDelayMs        = 1000;
	configLocation->packHardOverVoltage      = MODCONFIG_SPRIG_DEFAULT_PACK_HARD_OVER;  // Standard ESCs; raise only when every HV part is rated (D13)
	configLocation->sprigConfigMagic         = MODCONFIG_SPRIG_MAGIC;
}

void modConfigSprigApply(modConfigGeneralConfigStructTypedef *configLocation) {
#if SPRIG_FLIGHT_BUILD
	configLocation->sprigCanEnabled       = true;
	configLocation->canOpenChargerEnabled = false;
#endif
	if(configLocation->sprigCanEnabled) {
		// Both legacy status protocols use extended IDs that collide with DTI extended-mode packets.
		configLocation->emitStatusOverCAN = false;
		// The 0x0A23 keep-alive is gone; the DTI relay request replaces it.
		configLocation->useCANSafetyInput = false;
	}
}

static bool modConfigSprigInRange(float value, float min, float max) {
	return value >= min && value <= max;                                         // False for NaN
}

static bool modConfigSprigExpanderChannelValid(uint8_t channel) {
	return channel < SPRIG_EXPANDER_CHANNELS && channel != SPRIG_EXPANDER_READBACK_LOW && channel != SPRIG_EXPANDER_READBACK_HIGH;
}

bool modConfigSprigValid(const modConfigGeneralConfigStructTypedef *configLocation) {
	if(configLocation->sprigConfigMagic != MODCONFIG_SPRIG_MAGIC)
		return false;

	// DTI identity: required, standard node 1-30 but never 10, extended node 1-254.
	if(!libSprigDtiConfigValid(configLocation->dtiIdFormat, configLocation->dtiNodeId))
		return false;
	if(configLocation->dtiRelayRequestOutput < 1 || configLocation->dtiRelayRequestOutput > 4)
		return false;

	// Input expander, clear of the SSD1306 at 0x3C.
	if(configLocation->inputExpanderAddress != 0x20 && configLocation->inputExpanderAddress != 0x21)
		return false;
	if(!modConfigSprigExpanderChannelValid(configLocation->relayRequestInput) || !modConfigSprigExpanderChannelValid(configLocation->hvilInput))
		return false;
	if(configLocation->relayRequestInput == configLocation->hvilInput)
		return false;

	if(configLocation->dtiEnableOutput != dtiEnableOutputNone && configLocation->dtiEnableOutput != dtiEnableOutputCooling)
		return false;

	if(configLocation->relayRequestDebounceMs == 0 || configLocation->relayRequestDebounceMs > SPRIG_MAX_DEBOUNCE_MS)
		return false;
	if(configLocation->dtiWatchTimeoutMs == 0 || configLocation->prechargeTimeoutMs == 0)
		return false;
	if(!modConfigSprigInRange(configLocation->prechargeMatchFraction, 0.5f, 1.0f))
		return false;
	if(!modConfigSprigInRange(configLocation->prechargeCrossCheckVolts, 0.1f, MODCONFIG_SPRIG_MAX_PACK_VOLTAGE))
		return false;

	// Current limits and the discharge trip (D8).
	if(!modConfigSprigInRange(configLocation->maxDischargeCurrent, 0.1f, 6553.4f))
		return false;
	if(!modConfigSprigInRange(configLocation->maxChargeCurrent, 0.1f, 6553.4f))
		return false;
	if(!modConfigSprigInRange(configLocation->dischargeTripCurrent, 0.1f, 1.0e5f))
		return false;
	if(configLocation->dischargeTripDelayMs == 0)
		return false;
	if(!modConfigSprigInRange(configLocation->chargeTripCurrent, 0.1f, 1.0e5f))
		return false;
	if(configLocation->chargeTripDelayMs == 0)
		return false;

	// The pack-total limit may not exceed the Master-HV design rating (D13, operator: 1000 V).
	if(!modConfigSprigInRange(configLocation->packHardOverVoltage, 1.0f, MODCONFIG_SPRIG_MAX_PACK_VOLTAGE))
		return false;

	// The CANopen charger path listens on 0x048A, the standard broadcast of DTI node 10.
	if(configLocation->canOpenChargerEnabled && configLocation->dtiIdFormat == SPRIG_DTI_FORMAT_STANDARD && configLocation->dtiNodeId == SPRIG_DTI_STD_NODE_RESERVED)
		return false;

	return true;
}
