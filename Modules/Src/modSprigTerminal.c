/*
	USB terminal command "sprig". See modSprigTerminal.h.

	  sprig                      show the Sprig configuration and status
	  sprig set <field> <value>  change a field (relays open only), then "sprig store"
	  sprig store                store the configuration (increments config_revision)
	  sprig maint on|off         enter or leave MAINTENANCE (STANDBY, relays open)
 */

#include "modSprigTerminal.h"
#include "modSprig.h"
#include "modTerminal.h"
#include "modCommands.h"
#include "libSprigCan.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
	SPRIG_FIELD_U8 = 0,
	SPRIG_FIELD_U32,
	SPRIG_FIELD_FLOAT
} modSprigTerminalFieldTypeTypedef;

typedef struct {
	const char *name;
	size_t      offset;
	modSprigTerminalFieldTypeTypedef type;
} modSprigTerminalFieldTypedef;

#define SPRIG_FIELD(field, type) {#field, offsetof(modConfigGeneralConfigStructTypedef, field), type}

static const modSprigTerminalFieldTypedef modSprigTerminalFields[] = {
	SPRIG_FIELD(sprigCanEnabled,          SPRIG_FIELD_U8),
	SPRIG_FIELD(dtiIdFormat,              SPRIG_FIELD_U8),
	SPRIG_FIELD(dtiNodeId,                SPRIG_FIELD_U8),
	SPRIG_FIELD(dtiRelayRequestOutput,    SPRIG_FIELD_U8),
	SPRIG_FIELD(inputExpanderAddress,     SPRIG_FIELD_U8),
	SPRIG_FIELD(relayRequestInput,        SPRIG_FIELD_U8),
	SPRIG_FIELD(hvilInput,                SPRIG_FIELD_U8),
	SPRIG_FIELD(dtiEnableOutput,          SPRIG_FIELD_U8),
	SPRIG_FIELD(canOpenChargerEnabled,    SPRIG_FIELD_U8),
	SPRIG_FIELD(relayRequestDebounceMs,   SPRIG_FIELD_U32),
	SPRIG_FIELD(dtiWatchTimeoutMs,        SPRIG_FIELD_U32),
	SPRIG_FIELD(prechargeTimeoutMs,       SPRIG_FIELD_U32),
	SPRIG_FIELD(prechargeMatchFraction,   SPRIG_FIELD_FLOAT),
	SPRIG_FIELD(prechargeCrossCheckVolts, SPRIG_FIELD_FLOAT),
	SPRIG_FIELD(maxDischargeCurrent,      SPRIG_FIELD_FLOAT),
	SPRIG_FIELD(maxChargeCurrent,         SPRIG_FIELD_FLOAT),
	SPRIG_FIELD(dischargeTripCurrent,     SPRIG_FIELD_FLOAT),
	SPRIG_FIELD(dischargeTripDelayMs,     SPRIG_FIELD_U32),
	SPRIG_FIELD(chargeTripCurrent,        SPRIG_FIELD_FLOAT),
	SPRIG_FIELD(chargeTripDelayMs,        SPRIG_FIELD_U32),
	SPRIG_FIELD(packHardOverVoltage,      SPRIG_FIELD_FLOAT),
};
#define SPRIG_TERMINAL_FIELDS (sizeof(modSprigTerminalFields) / sizeof(modSprigTerminalFields[0]))

static modConfigGeneralConfigStructTypedef *modSprigTerminalConfig;

static void modSprigTerminalPrintField(const modSprigTerminalFieldTypedef *field) {
	const uint8_t *base = (const uint8_t *)modSprigTerminalConfig + field->offset;

	switch(field->type) {
		case SPRIG_FIELD_U8:
			modCommandsPrintf("%-25s: %u", field->name, (unsigned)*base);
			break;
		case SPRIG_FIELD_U32: {
			uint32_t value;
			memcpy(&value, base, sizeof(value));
			modCommandsPrintf("%-25s: %lu", field->name, (unsigned long)value);
			} break;
		default: {
			float value;
			memcpy(&value, base, sizeof(value));
			modCommandsPrintf("%-25s: %.3f", field->name, (double)value);
			} break;
	}
}

static bool modSprigTerminalSetField(const char *name, const char *text) {
	char *end;

	for(size_t i = 0; i < SPRIG_TERMINAL_FIELDS; i++) {
		const modSprigTerminalFieldTypedef *field = &modSprigTerminalFields[i];
		uint8_t *base = (uint8_t *)modSprigTerminalConfig + field->offset;

		if(strcmp(name, field->name) != 0)
			continue;

		if(field->type == SPRIG_FIELD_FLOAT) {
			float value = strtof(text, &end);
			if(end == text || *end != '\0')
				return false;
			memcpy(base, &value, sizeof(value));
		}else{
			unsigned long value = strtoul(text, &end, 0);                            // Accepts 0x.. for addresses
			if(end == text || *end != '\0')
				return false;
			if(field->type == SPRIG_FIELD_U8) {
				if(value > 0xFF)
					return false;
				*base = (uint8_t)value;
			}else{
				uint32_t value32 = (uint32_t)value;
				memcpy(base, &value32, sizeof(value32));
			}
		}
		modConfigSprigApply(modSprigTerminalConfig);
		return true;
	}

	return false;
}

static void modSprigTerminalShow(void) {
	modCommandsPrintf("-----   Sprig BMS CAN v1   -----");
	for(size_t i = 0; i < SPRIG_TERMINAL_FIELDS; i++)
		modSprigTerminalPrintField(&modSprigTerminalFields[i]);
	modCommandsPrintf("%-25s: %u", "configRevision", (unsigned)modSprigTerminalConfig->configRevision);
	modCommandsPrintf("%-25s: %s", "configuration", modConfigSprigValid(modSprigTerminalConfig) ? "valid" : "INVALID (faults_b bit 4)");
	modCommandsPrintf("%-25s: %s", "maintenance", modSprigMaintenance() ? "on" : "off");
	modCommandsPrintf("%-25s: 0x%08lX", "firmware build", (unsigned long)SPRIG_FIRMWARE_BUILD);
	modCommandsPrintf("dtiIdFormat: 0 standard, 1 extended, 255 unset. dtiNodeId 255 = unset.");
	modCommandsPrintf("Expander channels are port*8+pin; 6 and 7 are the readback straps.");
	modCommandsPrintf(" ");
}

static void modSprigTerminalCommand(int argc, const char **argv) {
	if(argc == 1) {
		modSprigTerminalShow();
	}else if(argc == 3 && strcmp(argv[1], "maint") == 0) {
		bool enable = strcmp(argv[2], "on") == 0;
		if(!enable && strcmp(argv[2], "off") != 0)
			modCommandsPrintf("Usage: sprig maint on|off");
		else if(modSprigSetMaintenance(enable))
			modCommandsPrintf("Maintenance %s", enable ? "on: CAN configuration channel open, relays will not close" : "off");
		else
			modCommandsPrintf("Refused: maintenance needs STANDBY with every relay open");
	}else if(argc == 4 && strcmp(argv[1], "set") == 0) {
		if(!modSprigRelaysOpen())
			modCommandsPrintf("Refused: relays are closed");
		else if(modSprigTerminalSetField(argv[2], argv[3]))
			modCommandsPrintf("%s set; \"sprig store\" to keep it", argv[2]);
		else
			modCommandsPrintf("Unknown field or bad value");
	}else if(argc == 2 && strcmp(argv[1], "store") == 0) {
		if(!modSprigRelaysOpen())
			modCommandsPrintf("Refused: relays are closed");
		else if(modConfigStoreConfig())
			modCommandsPrintf("Stored, config revision %u", (unsigned)modSprigTerminalConfig->configRevision);
		else
			modCommandsPrintf("Store failed");
	}else{
		modCommandsPrintf("Usage: sprig | sprig set <field> <value> | sprig store | sprig maint on|off");
	}
}

void modSprigTerminalInit(modConfigGeneralConfigStructTypedef *generalConfigPointer) {
	modSprigTerminalConfig = generalConfigPointer;
	modTerminalRegisterCommandCallBack("sprig", "Sprig BMS CAN v1 configuration and maintenance.", "[set <field> <value> | store | maint on|off]", modSprigTerminalCommand);
}
