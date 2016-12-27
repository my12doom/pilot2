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
	int update(const float q[4], const float acc_body[3], devices::gps_data gps, sensors::flow_data flow, float sonar, float baro, float dt, bool armed = false, bool airborne = true);
	void set_gps_latency(int new_latency){latency = new_latency;}
	int state();

	CircularQueue<matrix, 20> history_pos;
	int64_t last_history_push;
	int latency;
	double home_lat;
	double home_lon;
	float ticker;
	bool position_healthy;

	bool sonar_healthy;
	float sonar_ticker;
	float last_valid_sonar;
	float saturation_timeout;

	matrix P;
	matrix x; // {pos_ned[3], vel_ned[3], acc_bias_body[3], vel_bias_ned[3], sonar_surface_height}
	matrix Q;
	matrix R;
	float acc_ned[3];

	float gps_north;
	float gps_east;

	float gyro[3];
	float local[2];
	float vx;
	float vy;
	float vx_lpf;
	float vy_lpf;
	float predict_flow[2];
	float v_hbf[2];
	float v_bf[3];
	float baro_comp;

	bool flow_healthy;
	float flow_ticker;
	int _state;	// 0: not ready, 1: flow, 2: transiting, 3:gps


	float co[3];
};