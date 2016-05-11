// yet another filter for accelerometer - GPS fusion

#pragma once

#include <stdint.h>
#include <Protocol/common.h>
#include <utils/fifo.h>
#include <math/matrix.h>
#include <HAL/Interface/IGPS.h>
#include <HAL/Interface/IFlow.h>


class pos_estimator2
{
public:
	pos_estimator2();
	~pos_estimator2();

	int reset();		// mainly for after GPS glitch handling
	int update(const float q[4], const float acc_body[3], devices::gps_data gps, float baro, float dt);
	void set_gps_latency(int new_latency){latency = new_latency;}
	int state();

	CircularQueue<matrix, 20> history_pos;
	int64_t last_history_push;
	int latency;
	double home_lat;
	double home_lon;
	float ticker;
	bool position_healthy;

	matrix P;
	matrix x; // {pos_ned[3], vel_ned[3], acc_bias_body[3], vel_bias_ned[3]}
	matrix Q;
	matrix R;
	float acc_ned[3];

	float gps_north;
	float gps_east;

	float gyro[3];
	sensors::px4flow_frame frame;
	float vx;
	float vy;
	float predict_flow[2];
	float v_hbf[2];

	bool flow_healthy;
	float flow_ticker;
	int _state;	// 0: not ready, 1: flow, 2: transiting, 3:gps
};