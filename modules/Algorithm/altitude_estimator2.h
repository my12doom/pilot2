#pragma once

#include <stdint.h>
#include <modules/math/matrix.h>

class altitude_estimator2
{
public:
	altitude_estimator2();
	~altitude_estimator2();

	// pass NAN to baro to indicate that baro data is not ready.
	int update(float accelz, float baro, float sonar, float dt);

	// set to true to lower baro factor for land effect compensating
	void set_land_effect(bool compensate_land_effect){this->compensate_land_effect = compensate_land_effect;}

	// set static_mode to true to tell the estimator that the machine is not flying, and estimator should trust more baro data.
	void set_static_mode(bool static_mode){this->static_mode = static_mode;}

	matrix x;
protected:
	bool static_mode;
	bool compensate_land_effect;
	matrix P;
	bool sonar_used;
	float last_valid_sonar;
	float sonar_ticker;
};