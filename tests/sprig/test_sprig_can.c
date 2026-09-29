/*
	Host unit test: Sprig BMS CAN v1 encoder against the authoritative vector file
	(Sprig-Flight-Director docs/architecture/sprig-bms-can-v1-vectors.tsv, vendored in
	tests/sprig/vectors/ and pinned to a Flight Director commit; see tests/sprig/vectors/SOURCE).

	Every "decoded" row is encoded from its physical field values by the firmware's own encoder
	(libSprigCan.c), through the same scaling modSprig.c uses, and must match byte for byte.
	Change the vector file, not this code.
 */

#include "sprig_test.h"
#include "libSprigCan.h"
#include <stdlib.h>
#include <string.h>

#define VECTOR_LINE_MAX   1024
#define VECTOR_COLUMNS    7

typedef struct {
	char    *caseName;
	char    *id;
	char    *counter;
	char    *frame;
	char    *expect;
	char    *lost;
	char    *fields;
} vectorRowTypedef;

static int vectorFieldsFound;

// Value of "key=value" in the space-separated fields column; a "!invalid" suffix is stripped because
// the encoder still has to produce the raw reserved value. Returns NULL if the key is absent.
static const char *vectorField(const char *fields, const char *key) {
	static char value[64];
	size_t keyLength = strlen(key);
	const char *p = fields;

	while(*p) {
		while(*p == ' ')
			p++;
		if(strncmp(p, key, keyLength) == 0 && p[keyLength] == '=') {
			const char *start = p + keyLength + 1;
			size_t n = strcspn(start, " !");
			if(n >= sizeof(value))
				n = sizeof(value) - 1;
			memcpy(value, start, n);
			value[n] = '\0';
			vectorFieldsFound++;
			return value;
		}
		p += strcspn(p, " ");
	}
	return NULL;
}

static float vectorFloat(const char *fields, const char *key) {
	const char *value = vectorField(fields, key);
	if(!value || strcmp(value, "unavailable") == 0)
		return NAN;                                                                  // The spec sentinel: the firmware encodes NaN as unavailable
	return strtof(value, NULL);
}

static unsigned long vectorInt(const char *fields, const char *key) {
	const char *value = vectorField(fields, key);
	return value ? strtoul(value, NULL, 0) : 0xFFFFFFFFul;                          // 0x.. and decimal
}

static int vectorFrameBytes(const char *text, uint8_t *bytes) {
	int count = 0;
	char *end;

	while(*text && count < 16) {
		unsigned long value = strtoul(text, &end, 16);
		if(end == text)
			break;
		bytes[count++] = (uint8_t)value;
		text = end;
	}
	return count;
}

// Encode one decoded row with the firmware encoder, using the scaling from modSprigBuildFrame().
static bool vectorEncode(uint16_t id, uint8_t counter, const char *f, uint8_t frame[SPRIG_CAN_DLC]) {
	switch(id) {
		case SPRIG_CAN_ID_STATE: {
			libSprigCanStateFrameTypedef s = {(uint8_t)vectorInt(f, "state"), (uint8_t)vectorInt(f, "relay_outputs"), (uint8_t)vectorInt(f, "permissions"),
			                                  (uint8_t)vectorInt(f, "faults_a"), (uint8_t)vectorInt(f, "faults_b"), (uint8_t)vectorInt(f, "last_open_reason")};
			libSprigCanPackState(&s, counter, frame);
			} return true;
		case SPRIG_CAN_ID_PACK: {
			libSprigCanPackFrameTypedef p;
			p.packVoltageDeciVolt = (uint16_t)libSprigCanScale(vectorFloat(f, "pack_voltage"), 10.0f, 0, 0xFFFF, 0);
			p.packCurrentDeciAmp  = (int16_t)libSprigCanScale(vectorFloat(f, "pack_current"), 10.0f, -32768, 32767, 0);   // Spec sign: + = discharge
			p.busVoltageDeciVolt  = (uint16_t)libSprigCanScale(vectorFloat(f, "bus_voltage"), 10.0f, 0, 0xFFFE, SPRIG_CAN_BUS_VOLTAGE_UNAVAILABLE);
			libSprigCanPackPack(&p, counter, frame);
			} return true;
		case SPRIG_CAN_ID_LIMITS: {
			libSprigCanLimitsFrameTypedef l;
			l.dischargeCurrentLimitDeciAmp = (uint16_t)libSprigCanScale(vectorFloat(f, "discharge_current_limit"), 10.0f, 0, 0xFFFF, 0);
			l.chargeCurrentLimitDeciAmp    = (uint16_t)libSprigCanScale(vectorFloat(f, "charge_current_limit"), 10.0f, 0, 0xFFFF, 0);
			l.stateOfChargeCentiPercent    = (uint16_t)libSprigCanScale(vectorFloat(f, "state_of_charge"), 100.0f, 0, 10000, 0);
			libSprigCanPackLimits(&l, counter, frame);
			} return true;
		case SPRIG_CAN_ID_CELLS: {
			// The file gives millivolts; the firmware scales volts x 1000.
			libSprigCanCellsFrameTypedef c;
			c.cellVoltageMinMilliVolt = (uint16_t)libSprigCanScale(vectorFloat(f, "cell_voltage_min") / 1000.0f, 1000.0f, 0, 0xFFFF, 0);
			c.cellVoltageMaxMilliVolt = (uint16_t)libSprigCanScale(vectorFloat(f, "cell_voltage_max") / 1000.0f, 1000.0f, 0, 0xFFFF, 0);
			c.cellIndexMin = (uint8_t)vectorInt(f, "cell_index_min");
			c.cellIndexMax = (uint8_t)vectorInt(f, "cell_index_max");
			libSprigCanPackCells(&c, counter, frame);
			} return true;
		case SPRIG_CAN_ID_TEMPS: {
			libSprigCanTempsFrameTypedef t;
			t.cellTempMaxDeciC = (int16_t)libSprigCanScale(vectorFloat(f, "cell_temp_max"), 10.0f, -32768, 32767, 0);
			t.cellTempAvgDeciC = (int16_t)libSprigCanScale(vectorFloat(f, "cell_temp_avg"), 10.0f, -32768, 32767, 0);
			t.bmsTempMaxC      = (int8_t)libSprigCanScale(vectorFloat(f, "bms_temp_max"), 1.0f, -128, 127, 0);
			libSprigCanPackTemps(&t, counter, frame);
			} return true;
		case SPRIG_CAN_ID_ENERGY: {
			libSprigCanEnergyFrameTypedef e;
			e.remainingCapacityCentiAh = (uint16_t)libSprigCanScale(vectorFloat(f, "remaining_capacity"), 100.0f, 0, 0xFFFF, 0);
			e.fullCapacityCentiAh      = (uint16_t)libSprigCanScale(vectorFloat(f, "full_capacity"), 100.0f, 0, 0xFFFF, 0);
			e.seriesCells              = (uint8_t)vectorInt(f, "series_cells");
			libSprigCanPackEnergy(&e, counter, frame);
			} return true;
		case SPRIG_CAN_ID_IDENT: {
			libSprigCanIdentFrameTypedef i = {(uint32_t)vectorInt(f, "firmware_build"), (uint8_t)vectorInt(f, "config_revision"), (uint8_t)vectorInt(f, "hardware_id")};
			libSprigCanPackIdent(&i, counter, frame);
			} return true;
		default:
			return false;
	}
}

static bool vectorCrcValid(uint16_t id, const uint8_t *frame) {
	uint8_t crcInput[9] = {(uint8_t)(id >> 8), (uint8_t)id};
	memcpy(&crcInput[2], frame, 7);
	return libSprigCanCrc8(crcInput, 9) == frame[7];
}

static void testVectorRow(const vectorRowTypedef *row) {
	char name[160];
	uint8_t expected[16], frame[SPRIG_CAN_DLC];
	int length = vectorFrameBytes(row->frame, expected);
	bool standard = strncmp(row->id, "std:", 4) == 0;
	uint16_t id = (uint16_t)strtoul(row->id + 4, NULL, 16);

	if(strcmp(row->expect, "decoded") == 0) {
		snprintf(name, sizeof(name), "vector %s: encoder reproduces %s#%s", row->caseName, row->id, row->frame);
		vectorFieldsFound = 0;
		bool encoded = standard && length == SPRIG_CAN_DLC && vectorEncode(id, (uint8_t)strtoul(row->counter, NULL, 10), row->fields, frame);
		bool same = encoded && memcmp(frame, expected, SPRIG_CAN_DLC) == 0;
		if(encoded && !same)
			printf("  got      %02X %02X %02X %02X %02X %02X %02X %02X\n", frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7]);
		SPRIG_CHECK(name, same && vectorFieldsFound >= 3);
	}else if(strcmp(row->expect, "bad_crc") == 0) {
		snprintf(name, sizeof(name), "vector %s: the firmware CRC rejects it", row->caseName);
		SPRIG_CHECK(name, length == SPRIG_CAN_DLC && !vectorCrcValid(id, expected));
	}else if(strcmp(row->expect, "wrong_version") == 0) {
		snprintf(name, sizeof(name), "vector %s: CRC valid, only the version differs", row->caseName);
		SPRIG_CHECK(name, length == SPRIG_CAN_DLC && vectorCrcValid(id, expected) && (expected[6] >> 4) != SPRIG_CAN_PROTOCOL_VERSION);
	}else if(strcmp(row->expect, "ignored") == 0 && standard && id == SPRIG_CAN_ID_FORBIDDEN) {
		snprintf(name, sizeof(name), "vector %s: 0x%03X is the ID the firmware never transmits", row->caseName, id);
		SPRIG_CHECK(name, id == SPRIG_CAN_ID_FORBIDDEN);
	}
	// wrong_length, repeated_counter and not_mine are decoder-side cases.
}

static void testVectorFile(const char *path) {
	char line[VECTOR_LINE_MAX];
	int rows = 0, decodedRows = 0;
	FILE *file = fopen(path, "r");

	SPRIG_CHECK("vector file opens", file != NULL);
	if(!file)
		return;

	while(fgets(line, sizeof(line), file)) {
		char *columns[VECTOR_COLUMNS];
		int count = 0;
		char *p = line;

		line[strcspn(line, "\r\n")] = '\0';
		if(line[0] == '#' || line[0] == '\0')
			continue;

		while(count < VECTOR_COLUMNS) {
			columns[count++] = p;
			p = strchr(p, '\t');
			if(!p)
				break;
			*p++ = '\0';
		}
		SPRIG_CHECK("vector row has 7 tab-separated columns", count == VECTOR_COLUMNS);
		if(count != VECTOR_COLUMNS)
			continue;

		vectorRowTypedef row = {columns[0], columns[1], columns[2], columns[3], columns[4], columns[5], columns[6]};
		rows++;
		if(strcmp(row.expect, "decoded") == 0)
			decodedRows++;
		testVectorRow(&row);
	}
	fclose(file);

	printf("  vector file: %d rows, %d decoded\n", rows, decodedRows);
	SPRIG_CHECK("vector file has decoded rows", decodedRows > 0);
}

static void testCrc(void) {
	const uint8_t check[] = "123456789";
	SPRIG_CHECK("CRC-8/SAE-J1850 check value 0x4B", libSprigCanCrc8(check, 9) == 0x4B);
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

int main(int argc, char **argv) {
	const char *vectorsPath = (argc > 1) ? argv[1] : "tests/sprig/vectors/sprig-bms-can-v1-vectors.tsv";

	testCrc();
	testVectorFile(vectorsPath);
	testNegativeCases();
	testDti();
	return sprigTestSummary("test_sprig_can");
}
