/*
	USB terminal command "sprig": shows and sets the Sprig configuration and
	enters or leaves MAINTENANCE. The stock ENNOID tool has no fields for these.
 */

#ifndef MODSPRIGTERMINAL_H_
#define MODSPRIGTERMINAL_H_

#include "modConfig.h"

void modSprigTerminalInit(modConfigGeneralConfigStructTypedef *generalConfigPointer);

#endif
