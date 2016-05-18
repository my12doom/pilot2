#include "EKFINS.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef WIN32
#include <float.h>
static unsigned long pnan[2]={0xffffffff, 0x7fffffff};
static double NAN = *( double* )pnan;
#define isnan _isnan
#endif

EKFINS::EKFINS()
{
	latency = 400000;
	home_lat = NAN;
	home_lon = NAN;
	reset();
}

EKFINS::~EKFINS()
{

}

int EKFINS::reset()		// mainly for after GPS glitch handling
{
	history_pos.clear();
	last_history_push = 0;
	gps_ticker = 0;
	gps_healthy = false;
	flow_healthy = false;

	P = matrix::diag(19, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0);
	x = matrix(19,1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	return 0;
}

int EKFINS::healthy()
{
	if (!gps_healthy)
		return false;

	// TODO: residual norm and covariance checking

	return gps_healthy;
}


int EKFINS::update( const float gyro[3], const float acc_body[3], const float mag[3], devices::gps_data gps, sensors::px4flow_frame frame, float baro, float dt, bool armed, bool airborne)
{
	// flow switching
	bool use_flow = (frame.ground_distance > 0) && (frame.qual > 133);
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
	bool use_gps = (gps.fix == 3 && gps.position_accuracy_horizontal < 3.5) || (gps_healthy && gps.position_accuracy_horizontal < 7);

	// home
	if (use_gps && isnan(home_lat))
	{
		home_lat = gps.latitude;
		home_lon = gps.longitude;

		LOGE("EKFINS: home set to %f , %f\n", home_lat, home_lon);
	}

	// update "GPS healthy" state, and reset position covariance if needed
	if (!gps_healthy && use_gps)
	{
		gps_ticker += dt;

		if (gps_ticker == 0)
		{
			LOGE("EKFINS: GPS coming online, clearing position covariance\n");
			P[(P.m+1)*7] = 100;
			P[(P.m+1)*8] = 100;
		}

		if (gps_ticker > 3)
		{
			gps_healthy = true;
			gps_ticker = 0;

			LOGE("EKFINS: position healthy\n");
		}
	}
	else if (gps_healthy && !use_gps)
	{
		gps_ticker += dt;

		if (gps_ticker > 3)
		{
			gps_healthy = false;
			gps_ticker = 0;

			printf("EKFINS: position failed\n");

			P[0] = 100;
			P[P.m+1] = 100;
		}
	}
	else
	{
		gps_ticker = 0;
	}

	// motion detection
	vector va = {acc_body[0], acc_body[1], acc_body[2]};
	vector vg = {gyro[0], gyro[1], gyro[2]};
	motion_acc.new_data(va);
	motion_gyro.new_data(vg);
	bool still = motion_acc.get_average(NULL) > 100 && motion_gyro.get_average(NULL) > 100;


	// prepare matrices
	float q0q0 = x[0] * x[0];
	float q0q1 = x[0] * x[1];
	float q0q2 = x[0] * x[2];
	float q0q3 = x[0] * x[3];
	float q1q1 = x[1] * x[1];
	float q1q2 = x[1] * x[2];
	float q1q3 = x[1] * x[3];
	float q2q2 = x[2] * x[2];
	float q2q3 = x[2] * x[3];
	float q3q3 = x[3] * x[3];
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
	float dt2 = dt/2;
	const float *g = gyro;

	acc_ned[0] = r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2];
	acc_ned[1] = r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2];
	acc_ned[2] = -(r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2);

	acc_ned[0] += r[0]* x[6] + r[1] *x[7] + r[2] * x[8];
	acc_ned[1] += r[3]* x[6] + r[4] *x[7] + r[5] * x[8];
	acc_ned[2] += -(r[6]* x[6] + r[7] *x[7] + r[8] * x[8]);


	// process function
	matrix F = matrix(19,19,
		1.0, dt2*-g[0], dt2*-g[1], dt2*-g[2], dt2*-x[1], dt2*-x[2], dt2*-x[3], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
		dt2*g[0], 1.0, dt2*g[2], dt2*-g[1], dt2*x[0], dt2*-x[3], dt2*x[2], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
		dt2*g[1], dt2*-g[2], 1.0, dt2*g[0], dt2*x[3], dt2*x[0], dt2*-x[1], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
		dt2*g[2], dt2*g[1], dt2*-g[0], 1.0, dt2*-x[2], dt2*x[1], dt2*x[0], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, dt, 0.0, 0.0, dtsq_2*r[0], dtsq_2*r[1], dtsq_2*r[2], 0.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0, dt, 0.0, dtsq_2*r[3], dtsq_2*r[4], dtsq_2*r[5], 0.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0, 0.0, dt, -dtsq_2*r[6], -dtsq_2*r[7], -dtsq_2*r[8], 0.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, dt*r[0], dt*r[1], dt*r[2], 0.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, dt*r[3], dt*r[4], dt*r[5], 0.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, -dt*r[6], -dt*r[7], -dt*r[8], 0.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0,
		0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0
		);
	matrix Bu = matrix(19,1,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		dtsq_2 * (r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2]),
		dtsq_2 * (r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2]),
		-dtsq_2 * (r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2),
		dt * (r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2]),
		dt * (r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2]),
		-dt * (r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2),
		0.0,0.0,0.0,
		0.0,0.0,0.0
		);

	// reset observation helper variables
	zk.n = 1;
	zk.m = 0;
	H.n = F.n;
	H.m = 0;
	R_count = 0;

	// observation function and observation
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
		H = matrix(6,12,
			1.0, 0.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 1.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 0.0, 1.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0,
			0.0, 0.0, 0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0,
			0.0, 0.0, 0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0,
			0.0, 0.0, 0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,1.0);

		add_observation(60.0, pos_north, 1.0, 0.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
	}
	else
	{
		zk = matrix(3,1,baro, 0.0, 0.0);
		H = matrix(3,12,
			0.0,0.0,1.0, 0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0,
			0.0,0.0,0.0, 1.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0,
			0.0,0.0,0.0, 0.0,1.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0
			);
	}


	// prediction and correction
	float f1 = dt / 0.005f;		// 0.005: tuned dt.
	float f2 = dt*dt / (0.005f*0.005f);
	matrix Q = matrix::diag(19, 4e-3, 4e-3, 4e-6, 
		1e-4, 1e-4, 1e-6, 
		1e-7, 1e-7, 1e-7, 
		1e-7, 1e-7, 5e-7);
	matrix R = matrix::diag(R_count, R_diag);

	matrix x1 = F * x + Bu;
	matrix P1 = F * P * F.transpos() + Q;
	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();

	float residual = (zk - H*x1)[0];

	x = x1 + K*(zk - H*x1);
	P = (matrix(P1.m) - K*H) * P1;


	return 0;
}

void EKFINS::add_observation(float dev, float zk, ...)	// ...: colomn of observation matrix
{
	R_diag[R_count++] = dev;

	int dimension = H.n;
	int start = dimension*H.m;

	va_list vl;
	va_start(vl,zk);
	for(int i=0; i<dimension; i++)
		H[start + i] = va_arg(vl,double);
	va_end(vl);

	H.m ++;
}
