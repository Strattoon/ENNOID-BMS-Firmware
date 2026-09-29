#include "driverSWPCAL6416.h"

static bool driverSWPCAL6416WritePair(uint8_t address, uint8_t registerAddress, uint8_t port0, uint8_t port1) {
	uint8_t data[2] = {port0, port1};																											// The register pointer auto-increments to port 1
	return driverHWI2C1MemWrite(address, registerAddress, data, sizeof(data));
}

bool driverSWPCAL6416Init(uint8_t address, uint8_t DDR0, uint8_t DDR1, uint8_t PUER0, uint8_t PUER1, uint8_t PUDR0, uint8_t PUDR1) {
	bool ok = true;

	driverHWI2C1Init();																																					// No-op when the OLED already started the bus

	ok &= driverSWPCAL6416WritePair(address, PCAL6416_REG_OUTPUT_PORT0, 0x00, 0x00);					// Outputs low before any pin is made an output
	ok &= driverSWPCAL6416WritePair(address, PCAL6416_REG_PULL_SELECT_PORT0, PUDR0, PUDR1);		// Pull direction before enabling the pulls
	ok &= driverSWPCAL6416WritePair(address, PCAL6416_REG_PULL_ENABLE_PORT0, PUER0, PUER1);
	ok &= driverSWPCAL6416WritePair(address, PCAL6416_REG_CONFIG_PORT0, DDR0, DDR1);

	return ok;
}

bool driverSWPCAL6416ReadInputs(uint8_t address, uint16_t *inputs) {
	uint8_t data[2];

	if(!driverHWI2C1MemRead(address, PCAL6416_REG_INPUT_PORT0, data, sizeof(data)))
		return false;

	*inputs = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
	return true;
}

bool driverSWPCAL6416ReadConfig(uint8_t address, uint8_t registerAddress, uint16_t *value) {
	uint8_t data[2];

	if(!driverHWI2C1MemRead(address, registerAddress, data, sizeof(data)))
		return false;

	*value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
	return true;
}
