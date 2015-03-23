// legacy complementary filter altitude estimator
// changed interface to replace kalman filter based altitude estimator in place.

#pragma once

#include <stdint.h>

class altitude_estimatorCF
{
public:
	altitude_estimatorCF();
	~altitude_estimatorCF();

	float state[4];	// 4x1 matrix, altitude, climb, accel, accel_bias

	// state[0] : sum(_position_base, _position_correction) - corrected position estimate in meter - relative to the home location (_base_lat, _base_lon, 0)
	// state[1] : latest velocity estimate (integrated from accelerometer values) in meter/s
	// state[2] : latest acceleration in meter/s
	// state[3] : accelerometer corrections

	// pass NAN to baro to indicate that baro data is not ready.
	int update(float accelz, float baro, float dt);

	// set to true to lower baro factor for land effect compensating
	void set_land_effect(bool compensate_land_effect){this->compensate_land_effect = compensate_land_effect;}

protected:
	
	bool compensate_land_effect;		// not used at all
	float _position_error;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
	float _position_base;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
	float _position_correction;       // sum of corrections to _position_base from delayed 1st order samples in cm
	float _velocity_base;             // latest velocity estimate (integrated from accelerometer values) in cm/s
	float _velocity_correction;       // latest velocity estimate (integrated from accelerometer values) in cm/s

};