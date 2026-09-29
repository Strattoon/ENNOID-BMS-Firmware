/*
	Open-circuit-voltage state-of-charge estimate for a resting LiPo cell (Sprig spec, first boot after
	flashing). MaxAmps publishes no voltage-to-charge curve for the Graphene 5200 11S, so the end points
	are MaxAmps' published limits (3.00 V minimum = 0 %, 4.20 V maximum = 100 %; MA-5200-11s-Lipo-Pack
	data sheet) and the shape between them is a generic LiPo rest-voltage curve. Replace it with a
	bench-measured MaxAmps curve when one exists.
 */

#ifndef LIBSPRIGOCV_H_
#define LIBSPRIGOCV_H_

// Resting cell voltage in volts -> state of charge in percent (0-100), linear between table points.
float libSprigOcvStateOfCharge(float cellVoltage);

#endif
