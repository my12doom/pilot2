// complemetary filter for accelerometer - GPS fusion
// caller is responsible for providing accel in lon/lat axis.
// for performance reason we don't use degree for geographic position unit, which have only ~0.1 meter un-guaranteed resolution with double variable
// and doube variable is not an option for performance and compatibility reasons.
// so we use integer variable and 1/COORDTIMES degree as unit.
// int32_t can provide ~0.01meter resolution, int64_t is even better.

#pragma once

#include <stdint.h>
#include <Protocol/common.h>
#include <Protocol/px4flow.h>

// struct definitions
typedef struct
{
	double longtitude;		// unit: meter
	double latitude;		// unit: meter
	double vlongtitude;		// unit: meter/s
	double vlatitude;		// unit: meter/s
	int64_t time;
} flow_position_meter;

class flow_estimator
{
public:
	flow_estimator();
	~flow_estimator();
	int reset();
	int update_imu(float accel_hbf[3], float angular_rate_bf[3], int64_t timestamp);			// unit: meter/s^2
	int update_flow(px4flow_frame flow, int64_t timestamp);										// unit: degree

	position_meter get_estimation(){return est;}
	position_meter get_raw(){return meter_raw;}
	bool healthy();
	
protected:
	float accel_bias[2];
	position_meter est;
	position_meter meter_raw;
	bool sonar_valid;
	bool flow_valid;
};
