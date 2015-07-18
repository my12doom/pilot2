#include "pos_estimator.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

double _time_constant_xy = 2.5f;
double _k1_xy = 3 / _time_constant_xy;
double _k2_xy = 3 / (_time_constant_xy*_time_constant_xy);
double _k3_xy = 1 / (_time_constant_xy*_time_constant_xy*_time_constant_xy);


flow_estimator::flow_estimator()
{
	reset();
}

flow_estimator::~flow_estimator()
{

}

int flow_estimator::reset()		// mainly for after GPS glitch handling
{
	memset(&est, 0, sizeof(est));
	memset(&meter_raw, 0, sizeof(meter_raw));

	return 0;
}

int flow_estimator::update_imu(float accel_hbf[3], float angular_rate_bf[3], int64_t timestamp)
{
	// predict velocity

	// integrate position

	// surface distance prediction

	return 0;
}

int pos_estimator::update_flow(px4flow_frame flow, int64_t timestamp)
{
	// if sonar reading available?

	// surface distance step response

	// surface distance correction

	// is flow quality acceptable?

	// angular rate compensation

	// pixel to linear velocity

	// velocity correction

	return 0;
}


bool pos_estimator::healthy()
{
	return sonar_valid && flow_valid;
}
