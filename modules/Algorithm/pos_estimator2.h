// yet another filter for accelerometer - GPS fusion

#pragma once

#include <stdint.h>
#include <Protocol/common.h>
#include <utils/fifo.h>
#include <math/matrix.h>
#include <HAL/Interface/IGPS.h>


class pos_estimator2
{
public:
	pos_estimator2();
	~pos_estimator2();

	int reset();		// mainly for after GPS glitch handling
	int update(const float q[4], const float acc_body[3], devices::gps_data gps, float baro, float dt)
		;
	void set_gps_latency(int new_latency){latency = new_latency;}

	CircularQueue<matrix, 20> history_pos;
	int64_t last_history_push;
	int latency;
	bool use_gps;
	double home_lat;
	double home_lon;
	float gps_ticker;

	matrix P;
	matrix x; // {pos_ned[3], vel_ned[3], acc_bias_body[3], vel_bias_ned[3]}
	matrix Q;
	matrix R;
	float acc_ned[3];
};