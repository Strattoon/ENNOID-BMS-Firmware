/*
	Sprig BMS CAN protocol v1 frame codec. See libSprigCan.h.
 */

#include "libSprigCan.h"
#include <math.h>

#define SPRIG_DTI_STD_PACKET_SHIFT 5
#define SPRIG_DTI_STD_NODE_MASK    0x1F
#define SPRIG_DTI_EXT_PACKET_SHIFT 8
#define SPRIG_DTI_EXT_NODE_MASK    0xFF
#define SPRIG_DTI_DIGITAL_OUT_BIT0 20  // Packet 0x24: digital outputs 1-4 are bits 20-23

uint8_t libSprigCanCrc8(const uint8_t *data, size_t length) {
	uint8_t crc = 0xFF;

	for(size_t i = 0; i < length; i++) {
		crc ^= data[i];
		for(uint8_t bit = 0; bit < 8; bit++)
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
	}

	return crc ^ 0xFF;
}

void libSprigCanFinalize(uint16_t id, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	uint8_t crcInput[2 + 7];

	frame[6] = (uint8_t)((SPRIG_CAN_PROTOCOL_VERSION << 4) | (counter & 0x0F));

	crcInput[0] = (uint8_t)(id >> 8);
	crcInput[1] = (uint8_t)(id & 0xFF);
	for(uint8_t i = 0; i < 7; i++)
		crcInput[2 + i] = frame[i];

	frame[7] = libSprigCanCrc8(crcInput, sizeof(crcInput));
}

int32_t libSprigCanScale(float value, float stepsPerUnit, int32_t min, int32_t max, int32_t nanValue) {
	if(isnan(value))
		return nanValue;

	float scaled = roundf(value * stepsPerUnit);

	if(scaled <= (float)min)
		return min;
	if(scaled >= (float)max)
		return max;
	return (int32_t)scaled;
}

static void libSprigCanClear(uint8_t frame[SPRIG_CAN_DLC]) {
	for(uint8_t i = 0; i < SPRIG_CAN_DLC; i++)
		frame[i] = SPRIG_CAN_UNUSED_BYTE;
}

static void libSprigCanPutU16(uint8_t *frame, uint8_t index, uint16_t value) {
	frame[index]     = (uint8_t)(value >> 8);
	frame[index + 1] = (uint8_t)(value & 0xFF);
}

void libSprigCanPackState(const libSprigCanStateFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	libSprigCanClear(frame);
	frame[0] = in->state;
	frame[1] = in->relayOutputs;
	frame[2] = in->permissions;
	frame[3] = in->faultsA;
	frame[4] = in->faultsB;
	frame[5] = in->lastOpenReason;
	libSprigCanFinalize(SPRIG_CAN_ID_STATE, counter, frame);
}

void libSprigCanPackPack(const libSprigCanPackFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	libSprigCanClear(frame);
	libSprigCanPutU16(frame, 0, in->packVoltageDeciVolt);
	libSprigCanPutU16(frame, 2, (uint16_t)in->packCurrentDeciAmp);
	libSprigCanPutU16(frame, 4, in->busVoltageDeciVolt);
	libSprigCanFinalize(SPRIG_CAN_ID_PACK, counter, frame);
}

void libSprigCanPackLimits(const libSprigCanLimitsFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	libSprigCanClear(frame);
	libSprigCanPutU16(frame, 0, in->dischargeCurrentLimitDeciAmp);
	libSprigCanPutU16(frame, 2, in->chargeCurrentLimitDeciAmp);
	libSprigCanPutU16(frame, 4, in->stateOfChargeCentiPercent);
	libSprigCanFinalize(SPRIG_CAN_ID_LIMITS, counter, frame);
}

void libSprigCanPackCells(const libSprigCanCellsFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	libSprigCanClear(frame);
	libSprigCanPutU16(frame, 0, in->cellVoltageMinMilliVolt);
	libSprigCanPutU16(frame, 2, in->cellVoltageMaxMilliVolt);
	frame[4] = in->cellIndexMin;
	frame[5] = in->cellIndexMax;
	libSprigCanFinalize(SPRIG_CAN_ID_CELLS, counter, frame);
}

void libSprigCanPackTemps(const libSprigCanTempsFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	libSprigCanClear(frame);
	libSprigCanPutU16(frame, 0, (uint16_t)in->cellTempMaxDeciC);
	libSprigCanPutU16(frame, 2, (uint16_t)in->cellTempAvgDeciC);
	frame[4] = (uint8_t)in->bmsTempMaxC;
	libSprigCanFinalize(SPRIG_CAN_ID_TEMPS, counter, frame);
}

void libSprigCanPackEnergy(const libSprigCanEnergyFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	libSprigCanClear(frame);
	libSprigCanPutU16(frame, 0, in->remainingCapacityCentiAh);
	libSprigCanPutU16(frame, 2, in->fullCapacityCentiAh);
	frame[4] = in->seriesCells;
	libSprigCanFinalize(SPRIG_CAN_ID_ENERGY, counter, frame);
}

void libSprigCanPackIdent(const libSprigCanIdentFrameTypedef *in, uint8_t counter, uint8_t frame[SPRIG_CAN_DLC]) {
	libSprigCanClear(frame);
	frame[0] = (uint8_t)(in->firmwareBuild >> 24);
	frame[1] = (uint8_t)(in->firmwareBuild >> 16);
	frame[2] = (uint8_t)(in->firmwareBuild >> 8);
	frame[3] = (uint8_t)(in->firmwareBuild);
	frame[4] = in->configRevision;
	frame[5] = in->hardwareId;
	libSprigCanFinalize(SPRIG_CAN_ID_IDENT, counter, frame);
}

bool libSprigDtiConfigValid(uint8_t format, uint8_t node) {
	if(format == SPRIG_DTI_FORMAT_STANDARD)
		return (node >= 1) && (node <= 30) && (node != SPRIG_DTI_STD_NODE_RESERVED);
	if(format == SPRIG_DTI_FORMAT_EXTENDED)
		return (node >= 1) && (node <= 254);
	return false;
}

uint32_t libSprigDtiId(uint8_t format, uint8_t packet, uint8_t node) {
	if(format == SPRIG_DTI_FORMAT_EXTENDED)
		return ((uint32_t)packet << SPRIG_DTI_EXT_PACKET_SHIFT) | node;
	return ((uint32_t)packet << SPRIG_DTI_STD_PACKET_SHIFT) | node;
}

libSprigDtiResultTypedef libSprigDtiDecode(uint8_t format, uint8_t node, uint8_t digitalOutputNumber,
                                           uint32_t id, bool extended, uint8_t dlc, const uint8_t *data,
                                           libSprigDtiFrameTypedef *out) {
	uint32_t packet;
	uint32_t frameNode;

	if(!libSprigDtiConfigValid(format, node) || digitalOutputNumber < 1 || digitalOutputNumber > 4)
		return SPRIG_DTI_NOT_MINE;

	// The same packet/node arithmetic in the other ID format is another device.
	if(extended != (format == SPRIG_DTI_FORMAT_EXTENDED))
		return SPRIG_DTI_NOT_MINE;

	if(extended) {
		packet    = id >> SPRIG_DTI_EXT_PACKET_SHIFT;
		frameNode = id & SPRIG_DTI_EXT_NODE_MASK;
	}else{
		packet    = id >> SPRIG_DTI_STD_PACKET_SHIFT;
		frameNode = id & SPRIG_DTI_STD_NODE_MASK;
	}

	if(frameNode != node)
		return SPRIG_DTI_NOT_MINE;
	if(packet != SPRIG_DTI_PACKET_GENERAL_1 && packet != SPRIG_DTI_PACKET_GENERAL_3 && packet != SPRIG_DTI_PACKET_GENERAL_5)
		return SPRIG_DTI_NOT_MINE;
	if(dlc != SPRIG_CAN_DLC)
		return SPRIG_DTI_REJECTED;

	out->packet = (uint8_t)packet;
	switch(packet) {
		case SPRIG_DTI_PACKET_GENERAL_1:
			out->inputVoltageVolt = (int16_t)(((uint16_t)data[6] << 8) | data[7]);
			break;
		case SPRIG_DTI_PACKET_GENERAL_3:
			out->faultCode = data[4];
			break;
		default: {
			// DTI numbers bits from byte 0; within a byte, bit 0 is the least significant.
			uint8_t bitIndex = SPRIG_DTI_DIGITAL_OUT_BIT0 + (digitalOutputNumber - 1);
			out->digitalOutput = ((data[bitIndex / 8] >> (bitIndex % 8)) & 1u) != 0;
			} break;
	}

	return SPRIG_DTI_DECODED;
}
