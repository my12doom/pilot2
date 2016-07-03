#pragma once

#include <stdint.h>
#include <modules/math/matrix.h>

class battery_estimator
{
public:
	battery_estimator();
	~battery_estimator();

	int update(const float voltage, const float current, const float dt);
	float get_internal_voltage(){return x.data[0];}
	float get_internal_resistance(){return x.data[1];}
	float get_mah_consumed(){return mah;}

// protected:
	int64_t last_log;
	bool init;
	float mah;
	matrix x;
	matrix P;
};