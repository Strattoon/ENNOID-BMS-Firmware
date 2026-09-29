/*
	Sprig relay-path inputs:
	- DTI watch: DTI HV-550/850 status packets 0x20, 0x22 and 0x24 from the configured node.
	- Input expander: PCAL6416 on I2C1 carrying the hardwired DTI relay request and HVIL.
 */

#ifndef MODSPRIGINPUTS_H_
#define MODSPRIGINPUTS_H_

#include <stdint.h>
#include <stdbool.h>
#include "modConfig.h"

// Opto on pulls the expander pin low (open-collector output with a pull-up). Set to 0 for an
// active-high board. Confirm against the expander board design.
#ifndef SPRIG_EXPANDER_INPUT_ACTIVE_LOW
#define SPRIG_EXPANDER_INPUT_ACTIVE_LOW 1
#endif

typedef struct {
	bool    healthy;             // 0x20, 0x22 and 0x24 all decoded within dtiWatchTimeoutMs
	bool    canRequest;          // 0x24 configured digital output
	uint8_t faultCode;           // 0x22
	float   inputVoltage;        // 0x20, volts
} modSprigDtiWatchTypedef;

typedef struct {
	bool ok;                     // Expander acknowledges and reads back P0.6 = 0, P0.7 = 1
	bool request;                // Hardwired DTI digital output active
	bool hvilClosed;
} modSprigExpanderTypedef;

void modSprigInputsInit(modConfigGeneralConfigStructTypedef *generalConfigPointer);

// Interrupt context: called for every received CAN frame.
void modSprigInputsDtiReceive(uint32_t id, bool extended, uint8_t dlc, const uint8_t *data);
modSprigDtiWatchTypedef modSprigInputsDtiWatch(void);

// Main loop: polls the expander (INT is not wired).
void modSprigInputsExpanderTask(void);
modSprigExpanderTypedef modSprigInputsExpander(void);

#endif
