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
	latency = 400000;
	home_lat = NAN;
	home_lon = NAN;
	reset();
}

pos_estimator2::~pos_estimator2()
{

}

int pos_estimator2::reset()		// mainly for after GPS glitch handling
{
	history_pos.clear();
	last_history_push = 0;
	gps_ticker = 0;
	use_gps = false;

	P = matrix::diag(12, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0);
	x = matrix(12,1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	Q = matrix::diag(12, 4e-3, 4e-3, 4e-6, 
						1e-4, 1e-4, 1e-6, 
						1e-7, 1e-7, 1e-7, 
						1e-7, 1e-7, 1e-7);

	R = matrix::diag(6, 1.0, 1.0, 1.0, 1e-2, 1e-2, 1e+2);
	return 0;
}

bool pos_estimator2::healthy()
{
	return use_gps;
}


int pos_estimator2::update(const float q[4], const float acc_body[3], devices::gps_data gps, float baro, float dt)			// unit: meter/s
{
	// GPS switching
	if (!use_gps)
	{
		if (gps.fix == 3 && gps.position_accuracy_horizontal < 5)
			gps_ticker += dt;
		else
			gps_ticker = 0;

		if (gps_ticker > 5)
		{
			use_gps = true;
			gps_ticker = 0;

			printf("pos_estimator2: using GPS\n");
			if (isnan(home_lat))
			{
				home_lat = gps.latitude;
				home_lon = gps.longitude;
			}
		}
	}
	else
	{

		if (gps.position_accuracy_horizontal > 10)
			gps_ticker += dt;
		else
			gps_ticker = 0;

		if (gps_ticker > 3)
		{
			use_gps = false;
			gps_ticker = 0;

			printf("pos_estimator2: disabled GPS\n");
		}

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
	float dtsq_2 = dt*dt/2;
	acc_ned[0] = r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2];
	acc_ned[1] = r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2];
	acc_ned[2] = r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2;

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

	if (use_gps)
	{
		float latitude_to_meter = 40007000.0f/360;
		float longtitude_to_meter = 40007000.0f/360*cos(gps.latitude * PI / 180);

		float pos_north = (gps.latitude - home_lat) * latitude_to_meter;
		float pos_east = (gps.longitude - home_lon) * longtitude_to_meter;
		float yaw_gps = gps.direction * 2 * PI / 360.0f;
		float vel_north = cos(yaw_gps) * gps.speed;
		float vel_east = sin(yaw_gps) * gps.speed;

		zk = matrix(5,1,pos_north, pos_east, baro, vel_north, vel_east);
		R = matrix::diag(5, 60.0, 60.0, 60.0, 5.0, 5.0);
		H = matrix(5,12,
			1.0, 0.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 1.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 0.0, 1.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 0.0, 0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0,
			0.0, 0.0, 0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0);
	}
	else
	{
		zk = matrix(3,1,baro, 0.0, 0.0);
		H = matrix(3,12,
			0.0,0.0,1.0, 0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0,
			0.0,0.0,0.0, 1.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0,
			0.0,0.0,0.0, 0.0,1.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0
			);
		R = matrix::diag(3,60.0, 5.0, 5.0);
	}


	matrix x1 = F * x + Bu;
	matrix P1 = F * P * F.transpos() + Q;
	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();

	float residual = (zk - H*x1)[0];

	x = x1 + K*(zk - H*x1);
	P = (matrix(P1.m) - K*H) * P1;


	return 0;
}
