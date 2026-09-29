/*
	Open-circuit-voltage state-of-charge estimate. See libSprigOcv.h.
 */

#include "libSprigOcv.h"

typedef struct {
	float volts;
	float percent;
} libSprigOcvPointTypedef;

// Resting voltage per cell, ascending. End points: MaxAmps minimum and maximum; shape: generic LiPo.
static const libSprigOcvPointTypedef libSprigOcvTable[] = {
	{3.00f,   0.0f}, {3.61f,   5.0f}, {3.69f,  10.0f}, {3.71f,  15.0f}, {3.73f,  20.0f},
	{3.75f,  25.0f}, {3.77f,  30.0f}, {3.79f,  35.0f}, {3.80f,  40.0f}, {3.82f,  45.0f},
	{3.84f,  50.0f}, {3.85f,  55.0f}, {3.87f,  60.0f}, {3.91f,  65.0f}, {3.95f,  70.0f},
	{3.98f,  75.0f}, {4.02f,  80.0f}, {4.08f,  85.0f}, {4.11f,  90.0f}, {4.15f,  95.0f},
	{4.20f, 100.0f},
};
#define LIBSPRIGOCV_POINTS (sizeof(libSprigOcvTable) / sizeof(libSprigOcvTable[0]))

float libSprigOcvStateOfCharge(float cellVoltage) {
	if(!(cellVoltage > libSprigOcvTable[0].volts))                              // Also catches NaN
		return 0.0f;
	if(cellVoltage >= libSprigOcvTable[LIBSPRIGOCV_POINTS - 1].volts)
		return 100.0f;

	for(unsigned i = 1; i < LIBSPRIGOCV_POINTS; i++) {
		if(cellVoltage <= libSprigOcvTable[i].volts) {
			const libSprigOcvPointTypedef *low = &libSprigOcvTable[i - 1];
			const libSprigOcvPointTypedef *high = &libSprigOcvTable[i];
			return low->percent + (cellVoltage - low->volts) * (high->percent - low->percent) / (high->volts - low->volts);
		}
	}
	return 100.0f;
}
