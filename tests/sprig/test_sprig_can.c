/*
	Host unit test: Sprig BMS CAN v1 codec against the spec test vectors
	(Sprig-Flight-Director docs/architecture/sprig-bms-can-v1.md, "Test vectors").
	Every frame is produced by the firmware's own encoder (libSprigCan.c) from
	physical values, through the same scaling the firmware uses.
 */

#include "sprig_test.h"
#include "libSprigCan.h"
#include <string.h>

static void expectFrame(const char *name, const uint8_t *actual, const uint8_t expected[8]) {
	bool same = memcmp(actual, expected, 8) == 0;
	if(!same) {
		printf("  got      %02X %02X %02X %02X %02X %02X %02X %02X\n", actual[0], actual[1], actual[2], actual[3], actual[4], actual[5], actual[6], actual[7]);
		printf("  expected %02X %02X %02X %02X %02X %02X %02X %02X\n", expected[0], expected[1], expected[2], expected[3], expected[4], expected[5], expected[6], expected[7]);
	}
	SPRIG_CHECK(name, same);
}

static uint16_t deci(float value) { return (uint16_t)libSprigCanScale(value, 10.0f, 0, 0xFFFE, 0xFFFF); }
static int16_t  deciSigned(float value) { return (int16_t)libSprigCanScale(value, 10.0f, -32768, 32767, 0); }

static void testCrc(void) {
	const uint8_t check[] = "123456789";
	SPRIG_CHECK("CRC-8/SAE-J1850 check value 0x4B", libSprigCanCrc8(check, 9) == 0x4B);
}

static void testStateVectors(void) {
	uint8_t frame[8];
	libSprigCanStateFrameTypedef s;

	s = (libSprigCanStateFrameTypedef){SPRIG_STATE_ENERGIZED, SPRIG_RELAY_DISCHARGE | SPRIG_RELAY_DISCHARGE_NEGATIVE,
		SPRIG_PERM_DISCHARGE_ALLOWED | SPRIG_PERM_IN_SOA | SPRIG_PERM_DTI_WATCH_HEALTHY | SPRIG_PERM_CAN_REQUEST | SPRIG_PERM_HW_REQUEST, 0, 0, SPRIG_OPEN_NONE};
	libSprigCanPackState(&s, 5, frame);
	expectFrame("500 ENERGIZED counter 5", frame, (const uint8_t[]){0x03, 0x0A, 0x9D, 0x00, 0x00, 0x00, 0x15, 0x30});

	s = (libSprigCanStateFrameTypedef){SPRIG_STATE_PRECHARGE_FAILED, 0,
		SPRIG_PERM_DISCHARGE_ALLOWED | SPRIG_PERM_IN_SOA | SPRIG_PERM_DTI_WATCH_HEALTHY | SPRIG_PERM_CAN_REQUEST | SPRIG_PERM_HW_REQUEST, 0, SPRIG_FAULT_B_PRECHARGE_TIMEOUT, SPRIG_OPEN_PRECHARGE_FAILURE};
	libSprigCanPackState(&s, 6, frame);
	expectFrame("500 PRECHARGE_FAILED counter 6", frame, (const uint8_t[]){0x06, 0x00, 0x9D, 0x00, 0x01, 0x04, 0x16, 0x07});

	s = (libSprigCanStateFrameTypedef){SPRIG_STATE_ENERGIZED, SPRIG_RELAY_DISCHARGE | SPRIG_RELAY_DISCHARGE_NEGATIVE,
		SPRIG_PERM_DISCHARGE_ALLOWED | SPRIG_PERM_IN_SOA | SPRIG_PERM_DTI_WATCH_HEALTHY | SPRIG_PERM_CAN_REQUEST, 0, SPRIG_FAULT_B_REQUEST_DISAGREEMENT, SPRIG_OPEN_NONE};
	libSprigCanPackState(&s, 7, frame);
	expectFrame("500 ENERGIZED disagreement counter 7", frame, (const uint8_t[]){0x03, 0x0A, 0x1D, 0x00, 0x40, 0x00, 0x17, 0x49});

	s = (libSprigCanStateFrameTypedef){SPRIG_STATE_STANDBY, 0,
		SPRIG_PERM_DISCHARGE_ALLOWED | SPRIG_PERM_IN_SOA | SPRIG_PERM_DTI_WATCH_HEALTHY | SPRIG_PERM_CAN_REQUEST | SPRIG_PERM_HW_REQUEST, 0, SPRIG_FAULT_B_HVIL_OPEN, SPRIG_OPEN_NONE};
	libSprigCanPackState(&s, 8, frame);
	expectFrame("500 STANDBY HVIL open counter 8", frame, (const uint8_t[]){0x01, 0x00, 0x9D, 0x00, 0x20, 0x00, 0x18, 0xBD});
}

static void testPackVectors(void) {
	uint8_t frame[8];
	libSprigCanPackFrameTypedef p;

	p = (libSprigCanPackFrameTypedef){deci(388.4f), deciSigned(52.3f), deci(386.9f)};
	libSprigCanPackPack(&p, 9, frame);
	expectFrame("501 388.4 V +52.3 A bus 386.9 V counter 9", frame, (const uint8_t[]){0x0F, 0x2C, 0x02, 0x0B, 0x0F, 0x1D, 0x19, 0x77});

	p = (libSprigCanPackFrameTypedef){deci(390.1f), deciSigned(-12.5f), deci(NAN)};
	libSprigCanPackPack(&p, 10, frame);
	expectFrame("501 390.1 V -12.5 A bus unavailable counter 10", frame, (const uint8_t[]){0x0F, 0x3D, 0xFF, 0x83, 0xFF, 0xFF, 0x1A, 0xD6});
}

static void testOtherVectors(void) {
	uint8_t frame[8];

	libSprigCanLimitsFrameTypedef l = {deci(150.0f), deci(30.0f), (uint16_t)libSprigCanScale(76.25f, 100.0f, 0, 10000, 0)};
	libSprigCanPackLimits(&l, 0, frame);
	expectFrame("502 150 A / 30 A / SoC 76.25 % counter 0", frame, (const uint8_t[]){0x05, 0xDC, 0x01, 0x2C, 0x1D, 0xC9, 0x10, 0x66});

	libSprigCanCellsFrameTypedef c = {(uint16_t)libSprigCanScale(3.712f, 1000.0f, 0, 0xFFFF, 0), (uint16_t)libSprigCanScale(3.741f, 1000.0f, 0, 0xFFFF, 0), 17, 42};
	libSprigCanPackCells(&c, 15, frame);
	expectFrame("503 3.712 V @17 / 3.741 V @42 counter 15", frame, (const uint8_t[]){0x0E, 0x80, 0x0E, 0x9D, 0x11, 0x2A, 0x1F, 0xCF});

	libSprigCanTempsFrameTypedef t = {deciSigned(31.4f), deciSigned(28.9f), (int8_t)libSprigCanScale(41.0f, 1.0f, -128, 127, 0)};
	libSprigCanPackTemps(&t, 3, frame);
	expectFrame("504 31.4 / 28.9 / 41 C counter 3", frame, (const uint8_t[]){0x01, 0x3A, 0x01, 0x21, 0x29, 0xFF, 0x13, 0xB3});

	t = (libSprigCanTempsFrameTypedef){deciSigned(-5.2f), deciSigned(-7.0f), (int8_t)libSprigCanScale(-10.0f, 1.0f, -128, 127, 0)};
	libSprigCanPackTemps(&t, 4, frame);
	expectFrame("504 -5.2 / -7.0 / -10 C counter 4", frame, (const uint8_t[]){0xFF, 0xCC, 0xFF, 0xBA, 0xF6, 0xFF, 0x14, 0x80});
}

static void testNegativeCases(void) {
	uint8_t frame[8];
	libSprigCanStateFrameTypedef s = {SPRIG_STATE_ENERGIZED, SPRIG_RELAY_DISCHARGE | SPRIG_RELAY_DISCHARGE_NEGATIVE,
		SPRIG_PERM_DISCHARGE_ALLOWED | SPRIG_PERM_IN_SOA | SPRIG_PERM_DTI_WATCH_HEALTHY | SPRIG_PERM_CAN_REQUEST | SPRIG_PERM_HW_REQUEST, 0, 0, 0};
	libSprigCanPackState(&s, 5, frame);

	// A receiver recomputes the CRC over ID + bytes 0-6; any single flipped data bit must change it.
	bool allDetected = true;
	for(uint8_t bit = 0; bit < 56; bit++) {
		uint8_t copy[8];
		memcpy(copy, frame, 8);
		copy[bit / 8] ^= (uint8_t)(1u << (bit % 8));
		uint8_t crcInput[9] = {0x05, 0x00};
		memcpy(&crcInput[2], copy, 7);
		if(libSprigCanCrc8(crcInput, 9) == copy[7])
			allDetected = false;
	}
	SPRIG_CHECK("every single-bit flip in bytes 0-6 breaks the CRC", allDetected);

	// The same payload on another ID must not verify: the ID is inside the CRC.
	uint8_t crcInput[9] = {0x05, 0x01};
	memcpy(&crcInput[2], frame, 7);
	SPRIG_CHECK("ID is covered by the CRC", libSprigCanCrc8(crcInput, 9) != frame[7]);

	// The counter wraps within the low nibble and never touches the version.
	libSprigCanPackState(&s, 0x1F, frame);
	SPRIG_CHECK("counter wraps to 4 bits, version stays 1", frame[6] == 0x1F);
}

static void testDti(void) {
	libSprigDtiFrameTypedef out;
	const uint8_t general1[8] = {0x00, 0x00, 0x24, 0x5E, 0x00, 0x71, 0x01, 0x86};   // Manual vector: 390 V
	const uint8_t general3[8] = {0x01, 0x53, 0x01, 0x17, 0x00, 0xFF, 0xFF, 0xFF};   // Manual vector: fault 0
	uint8_t general5[8]       = {0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};   // Bit 20 set: digital output 1

	SPRIG_CHECK("DTI node 10 standard is invalid", !libSprigDtiConfigValid(SPRIG_DTI_FORMAT_STANDARD, 10));
	SPRIG_CHECK("DTI unset format is invalid", !libSprigDtiConfigValid(SPRIG_DTI_FORMAT_UNSET, 5));
	SPRIG_CHECK("DTI standard node 31 is invalid", !libSprigDtiConfigValid(SPRIG_DTI_FORMAT_STANDARD, 31));
	SPRIG_CHECK("DTI standard node 5 is valid", libSprigDtiConfigValid(SPRIG_DTI_FORMAT_STANDARD, 5));

	SPRIG_CHECK("0x20 node 5 standard ID is 0x405", libSprigDtiId(SPRIG_DTI_FORMAT_STANDARD, 0x20, 5) == 0x405);
	SPRIG_CHECK("0x20 decodes input voltage 390 V",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x405, false, 8, general1, &out) == SPRIG_DTI_DECODED && out.inputVoltageVolt == 390);
	SPRIG_CHECK("0x22 decodes fault code 0",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x445, false, 8, general3, &out) == SPRIG_DTI_DECODED && out.faultCode == 0);
	SPRIG_CHECK("0x24 digital output 1 set",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x485, false, 8, general5, &out) == SPRIG_DTI_DECODED && out.digitalOutput);
	SPRIG_CHECK("0x24 digital output 2 clear",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 2, 0x485, false, 8, general5, &out) == SPRIG_DTI_DECODED && !out.digitalOutput);
	general5[2] = 0x00;
	SPRIG_CHECK("0x24 digital output 1 clear",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x485, false, 8, general5, &out) == SPRIG_DTI_DECODED && !out.digitalOutput);
	SPRIG_CHECK("other node is not mine",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x406, false, 8, general1, &out) == SPRIG_DTI_NOT_MINE);
	SPRIG_CHECK("extended frame is not mine in standard mode",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x405, true, 8, general1, &out) == SPRIG_DTI_NOT_MINE);
	SPRIG_CHECK("packet 0x21 is not watched",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x425, false, 8, general1, &out) == SPRIG_DTI_NOT_MINE);
	SPRIG_CHECK("7-byte watched frame is rejected",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_STANDARD, 5, 1, 0x405, false, 7, general1, &out) == SPRIG_DTI_REJECTED);
	SPRIG_CHECK("extended 0x2022 decodes in extended mode",
		libSprigDtiDecode(SPRIG_DTI_FORMAT_EXTENDED, 0x22, 1, 0x2022, true, 8, general1, &out) == SPRIG_DTI_DECODED && out.inputVoltageVolt == 390);
}

int main(void) {
	testCrc();
	testStateVectors();
	testPackVectors();
	testOtherVectors();
	testNegativeCases();
	testDti();
	return sprigTestSummary("test_sprig_can");
}
