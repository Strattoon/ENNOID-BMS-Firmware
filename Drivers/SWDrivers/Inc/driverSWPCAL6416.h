#include "stm32f3xx_hal.h"
#include "stdbool.h"
#include "driverHWI2C1.h"

#ifndef __DRIVERSWPCAL6416_H
#define __DRIVERSWPCAL6416_H

// PCAL6416A 16-bit I2C I/O expander on I2C1 (shared with the SSD1306 OLED).
#define PCAL6416_ADDRESS_LOW              0x20	// ADDR pin low
#define PCAL6416_ADDRESS_HIGH             0x21	// ADDR pin high

#define PCAL6416_REG_INPUT_PORT0          0x00
#define PCAL6416_REG_OUTPUT_PORT0         0x02
#define PCAL6416_REG_CONFIG_PORT0         0x06	// 1 = input, 0 = output
#define PCAL6416_REG_PULL_ENABLE_PORT0    0x46	// 1 = pull enabled
#define PCAL6416_REG_PULL_SELECT_PORT0    0x48	// 1 = pull-up, 0 = pull-down

// All functions return true when every I2C transfer was acknowledged.
bool driverSWPCAL6416Init(uint8_t address, uint8_t DDR0, uint8_t DDR1, uint8_t PUER0, uint8_t PUER1, uint8_t PUDR0, uint8_t PUDR1);
bool driverSWPCAL6416ReadInputs(uint8_t address, uint16_t *inputs);	// Bit (port*8 + pin)
bool driverSWPCAL6416ReadConfig(uint8_t address, uint8_t registerAddress, uint16_t *value);

#endif
