#include "pos_estimator2.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifdef WIN32
#include <float.h>
static unsigned long pnan[2]={0xffffffff, 0x7fffffff};
static double NAN = *( double* )pnan;
#define isnan _isnan
#endif

pos_estimator2::pos_estimator2()
{
	flow_ticker = 0;
	latency = 400000;
	home_lat = NAN;
	home_lon = NAN;
	_state = 1;
	reset();
}

pos_estimator2::~pos_estimator2()
{

}

int pos_estimator2::reset()		// mainly for after GPS glitch handling
{
	history_pos.clear();
	last_history_push = 0;
	ticker = 0;
	position_healthy = false;
	flow_healthy = false;

	P = matrix::diag(12, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0);
	x = matrix(12,1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	Q = matrix::diag(12, 4e-3, 4e-3, 4e-6, 
						1e-4, 1e-4, 1e-6, 
						1e-7, 1e-7, 1e-7, 
						1e-7, 1e-7, 5e-7);

	R = matrix::diag(6, 1.0, 1.0, 1.0, 1e-2, 1e-2, 1e+2);
	return 0;
}

int pos_estimator2::state()
{
// 	if (!position_healthy)
// 		return 0;

	// TODO: residual norm and covariance checking
	if (!position_healthy && !flow_healthy)
		return 0;

	return _state;
}


int pos_estimator2::update(const float q[4], const float acc_body[3], devices::gps_data gps, float baro, float dt)			// unit: meter/s
{
	// flow switching
	bool use_flow = (frame.ground_distance > 0) && (frame.qual > 133);
	last_sonar = frame.ground_distance > 0 ? frame.ground_distance/1000.0f : last_sonar;
	if (!flow_healthy && use_flow)
	{
		flow_ticker += dt;

		if (flow_ticker > 3)
		{
			LOGE("pos_estimator2: using flow\n");
			flow_healthy = true;
		}
	}
	else if (flow_healthy && !use_flow)
	{
		flow_ticker += dt;

		if (flow_ticker > 1)
		{
			LOGE("pos_estimator2: flow failed\n");
			flow_healthy = false;
		}
	}
	else
	{
		if (flow_ticker > 1)
		{
			printf("...\n");
		}

		flow_ticker = 0;
	}

	// GPS switching
	bool use_gps = (gps.fix == 3 && gps.position_accuracy_horizontal < 3.5) || (position_healthy && gps.position_accuracy_horizontal < 7);

	// home
	if (use_gps && isnan(home_lat))
	{
		home_lat = gps.latitude;
		home_lon = gps.longitude;

		LOGE("pos_estimator2: home set to %f , %f\n", home_lat, home_lon);
	}

	// update "position healthy" state, and reset position covariance if needed
	if (!position_healthy && use_gps)
	{
		if (ticker == 0)
		{
			LOGE("pos_estimator2: GPS coming online, clearing position covariance\n");
			P[0] = 100;
			P[P.m+1] = 100;
		}

		ticker += dt;

		_state = 2;

		if (ticker > 3)
		{
			position_healthy = true;
			ticker = 0;

			LOGE("pos_estimator2: position healthy\n");

			_state = 3;
		}
	}
	else if (position_healthy && !use_gps)
	{
		ticker += dt;

		_state = 2;

		if (ticker > 3)
		{
			position_healthy = false;
			ticker = 0;

			printf("pos_estimator2: position failed\n");

			_state = 1;
		}
	}
	else
	{
		ticker = 0;
	}

	// prepare matrices
	float q0q0 = q[0] * q[0];
	float q0q1 = q[0] * q[1];
	float q0q2 = q[0] * q[2];
	float q0q3 = q[0] * q[3];
	float q1q1 = q[1] * q[1];
	float q1q2 = q[1] * q[2];
	float q1q3 = q[1] * q[3];
	float q2q2 = q[2] * q[2];
	float q2q3 = q[2] * q[3];
	float q3q3 = q[3] * q[3];
	float r[9] = 
	{
		q0q0 + q1q1 - q2q2 - q3q3,// 11
		2.f * (q1q2 - q0q3),	// 21
		2.f * (q1q3 + q0q2),	// 31
		2.f * (q1q2 + q0q3),	// 12
		q0q0 - q1q1 + q2q2 - q3q3,// 22
		2.f * (q2q3 - q0q1),	// 32
		2.f * (q1q3 - q0q2),	// 13
		2.f * (q2q3 + q0q1),	// 23
		q0q0 - q1q1 - q2q2 + q3q3,// 33
	};
	float yaw = atan2f(r[3], r[0]);
	float dtsq_2 = dt*dt/2;
	acc_ned[0] = r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2];
	acc_ned[1] = r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2];
	acc_ned[2] = -(r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2);

	acc_ned[0] += r[0]* x[6] + r[1] *x[7] + r[2] * x[8];
	acc_ned[1] += r[3]* x[6] + r[4] *x[7] + r[5] * x[8];
	acc_ned[2] += -(r[6]* x[6] + r[7] *x[7] + r[8] * x[8]);

	matrix F = matrix(12,12,
		1.0,0.0,0.0, dt, 0.0, 0.0, dtsq_2*r[0], dtsq_2*r[1], dtsq_2*r[2], 0.0, 0.0, 0.0,
		0.0,1.0,0.0, 0.0, dt, 0.0, dtsq_2*r[3], dtsq_2*r[4], dtsq_2*r[5], 0.0, 0.0, 0.0,
		0.0,0.0,1.0, 0.0, 0.0, dt, -dtsq_2*r[6], -dtsq_2*r[7], -dtsq_2*r[8], 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 1.0,0.0,0.0, dt*r[0], dt*r[1], dt*r[2], 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 0.0,1.0,0.0, dt*r[3], dt*r[4], dt*r[5], 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,1.0, -dt*r[6], -dt*r[7], -dt*r[8], 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,0.0,0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0
		);
	matrix Bu = matrix(12,1,
		dtsq_2 * (r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2]),
		dtsq_2 * (r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2]),
		-dtsq_2 * (r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2),
		dt * (r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2]),
		dt * (r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2]),
		-dt * (r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2),
		0.0,0.0,0.0,
		0.0,0.0,0.0
		);
	matrix zk;
	matrix H;

	float latitude_to_meter = 40007000.0f/360;
	float longtitude_to_meter = 40007000.0f/360*cos(gps.latitude * PI / 180);

	float pos_north = (gps.latitude - home_lat) * latitude_to_meter;
	float pos_east = (gps.longitude - home_lon) * longtitude_to_meter;
	float yaw_gps = gps.direction * 2 * PI / 360.0f;
	float vel_north = cos(yaw_gps) * gps.speed;
	float vel_east = sin(yaw_gps) * gps.speed;
	if (!isnan(home_lat))
	{
		gps_north = pos_north;
		gps_east = pos_east;
	}

	if (use_gps)
	{
		zk = matrix(6,1,pos_north, pos_east, baro, vel_north, vel_east, gps.climb_rate);
		R = matrix::diag(6, 60.0, 60.0, 60.0, 5.0, 5.0, 15.0);
		H = matrix(6,12,
			1.0, 0.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 1.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 0.0, 1.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 0.0, 0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0,
			0.0, 0.0, 0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0,
			0.0, 0.0, 0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,1.0);
	}
	else
	{
		float pixel_compensated_x = frame.pixel_flow_x_sum;
		float pixel_compensated_y = frame.pixel_flow_y_sum;

		bool saturation = fabs(gyro[0]) > 60 * PI / 180 || fabs(gyro[1]) > 60 * PI / 180;
		
		if (/*fabs(pixel_compensated_x) > 5 && */fabs(pixel_compensated_x) < 35 /*&& fabs(pixel_compensated_y)>5 */&& fabs(pixel_compensated_y)<35 && !saturation)
		{
			 pixel_compensated_x -= gyro[0] * 18000 / PI * 0.0028f;
			 pixel_compensated_y -= gyro[1] * 18000 / PI * 0.0028f;
		}
		

		float wx = pixel_compensated_x / 28.0f * 100 * PI / 180;
		float wy = pixel_compensated_y / 28.0f * 100 * PI / 180;

		vx = wx * last_sonar;
		vy = wy * last_sonar;

		if (!use_flow)
			vx = vy = 0;

		int count = use_flow ? 4 : 3;
		count = 3;

		zk = matrix(count,1,baro, vx, vy, last_sonar);
		H = matrix(count,12,
			0.0,0.0,1.0, 0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0,
			0.0,0.0,0.0, -r[1],-r[4],-r[7],  0.0,0.0,0.0,  0.0,0.0,0.0,
			0.0,0.0,0.0, r[0],r[3],r[6],  0.0,0.0,0.0,  0.0,0.0,0.0,				// strange flow coordinates
			0.0,0.0,1.0, 0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0
			);
		R = matrix::diag(count,600.0, saturation?50.0 : 5.0, saturation ? 50.0 : 5.0, 16.0);

	}


	matrix x1 = F * x + Bu;
	matrix P1 = F * P * F.transpos() + Q;
	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();
	matrix hx1 = H*x1;

	predict_flow[0] = hx1[1];
	predict_flow[1] = hx1[2];

	float cos_yaw = cos(yaw);
	float sin_yaw = sin(yaw);
	v_hbf[0] = cos_yaw * x[3] + sin_yaw * x[4];
	v_hbf[1] = -sin_yaw * x[3] + cos_yaw * x[4];

	// -->> vx ~= -v_hbf[1]
	// -->> vy ~= v_hbf[0]

	x = x1 + K*(zk - hx1);
	P = (matrix(P1.m) - K*H) * P1;


	return 0;
}
