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

#define SONAR_MAX 4.5f
#define SONAR_MIN 0.0f

bool check(const char *name, matrix x)
{
	for(int i=0; i<x.m*x.n;i++)
	{
		if (isnan(x[i]) || !_finite(x[i]))
		{
			printf("%s fail @ (%d,%d):%f\n", name, i/x.m, i%x.m, x[i]);
			return false;
		}
	}

	return true;
}

EKFINS::EKFINS()
{
	motion_acc.set_threshold(0.2);
	motion_gyro.set_threshold(5*PI/180);

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
// 	history_pos.clear();
// 	last_history_push = 0;
	gps_ticker = 0;
	gps_healthy = false;
	flow_healthy = false;
	sonar_ticker = 0;
	sonar_healthy = false;
	mag_healthy = true;
	inited = false;
	still_inited = false;

	P = matrix::diag(19,
		100.0, 100.0, 100.0, 100.0,
		10.0, 10.0, 10.0,
		100.0, 100.0, 100.0,
		100.0, 100.0, 100.0,
		100.0, 100.0, 100.0,
		100.0, 100.0, 100.0
		);
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

int EKFINS::update_mode(const float gyro[3], const float acc_body[3], const float mag[3], devices::gps_data gps, sensors::px4flow_frame frame, float baro, float dt, bool armed, bool airborne)
{
	// flow switching
	bool use_flow = (frame.ground_distance > 0) && (frame.qual > 133);
	if (!flow_healthy && use_flow)
	{
		flow_ticker += dt;

		if (flow_ticker > 3)
		{
			LOGE("EKFINS: using flow\n");
			flow_healthy = true;
		}
	}
	else if (flow_healthy && !use_flow)
	{
		flow_ticker += dt;

		if (flow_ticker > 1)
		{
			LOGE("EKFINS: flow failed\n");
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

	bool current_sonar_healthy = frame.ground_distance/1000.0f > SONAR_MIN && frame.ground_distance/1000.0f < SONAR_MAX;
	if (current_sonar_healthy)
		last_valid_sonar = frame.ground_distance/1000.0f;
	if (sonar_healthy && !current_sonar_healthy)
	{
		sonar_ticker += dt;

		if (sonar_ticker > 0.5)
		{
			sonar_healthy = false;
			sonar_ticker = 0;
		}
	}
	else if (!sonar_healthy && current_sonar_healthy)
	{
		sonar_ticker += dt;
		if (sonar_ticker > 1)
		{
			sonar_healthy = true;
			sonar_ticker = 0;
			// TODO: reset sonar surface and covariance
		}
	}
	else
	{
		sonar_ticker = 0;
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

		if (gps_ticker == 0)
		{
			LOGE("EKFINS: GPS coming online, clearing position covariance\n");
			P[(P.m+1)*7] = 100;
			P[(P.m+1)*8] = 100;
		}

		gps_ticker += dt;

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
	still_inited = still = motion_acc.get_average(NULL) > 100 && motion_gyro.get_average(NULL) > 100;

	return 0;
}

int EKFINS::update(const float gyro[3], const float acc_body[3], const float mag[3], devices::gps_data gps, sensors::px4flow_frame frame, float baro, float dt, bool armed, bool airborne)
{
	if (!inited)
		init_attitude(acc_body, gyro, mag);

	if (update_mode(gyro, acc_body, mag, gps, frame, baro, dt, armed, airborne) < 0)
	{
		reset();
		return -1;
	}

	bool use_gps = (gps.fix == 3 && gps.position_accuracy_horizontal < 3.5) || (gps_healthy && gps.position_accuracy_horizontal < 7);
	bool use_flow = (frame.ground_distance > 0) && (frame.qual > 133);

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
	predicted_observation.n = 1;
	predicted_observation.m = 0;
	zk.n = 1;
	zk.m = 0;
	H.n = F.n;
	H.m = 0;
	R_count = 0;

	// prediction
	float f1 = dt / 0.005f;		// 0.005: tuned dt.
	float f2 = dt*dt / (0.005f*0.005f);
	matrix Q = matrix::diag(19,
		1e-8, 1e-8, 1e-8, 1e-8, 1e-12, 1e-12, 1e-12,
		4e-3, 4e-3, 4e-6, 
		1e-4, 1e-4, 1e-6, 
		1e-7, 1e-7, 1e-7, 
		1e-7, 1e-7, 5e-7);
	Q *= f1;
	matrix x1 = F * x + Bu;
	matrix P1 = F * P * F.transpos() + Q;


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


// 	acc and mag observation jacobian
// 			2*x[2], 2*-x[3], 2*x[0], 2*-x[1], 0.0, 0.0, 0.0,
// 				-2*x[1], 2*-x[0], 2*-x[3], 2*-x[2], 0.0, 0.0, 0.0,
// 				-2*x[0], 2*x[1], 2*x[2], -2*x[3], 0.0, 0.0, 0.0,
// 				2*x[0], 2*x[1], -2*x[2], -2*x[3], 0.0, 0.0, 0.0,
// 				-2*x[3], 2*x[2], 2*x[1], -2*x[0], 0.0, 0.0, 0.0,
// 				2*x[2], 2*x[3], 2*x[0], 2*x[1], 0.0, 0.0, 0.0,
// 	-2.f * (x1[1]*x1[3] - x1[0]*x1[2]),						// 13			acc[0]
// 		-2.f * (x1[2]*x1[3] + x1[0]*x1[1]),						// 23		acc[1]
// 		-(x1[0]*x1[0] - x1[1]*x1[1] - x1[2]*x1[2] + x1[3]*x1[3]),	// 33		acc[2]
// 		x[0]*x[0] + x[1]*x[1] - x[2]*x[2] - x[3]*x[3],		// 11		mag[0]
// 		2.f * (x[1]*x[2] - x[0]*x[3]),						// 21		mag[1]
// 		2.f * (x[1]*x[3] + x[0]*x[2]),						// 31		mag[2]

	// baro
	add_observation(60.0, baro, x1[9], 0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);


	float mag_0z[3] = {mag[0], mag[1], mag[2]};
	remove_mag_ned_z(mag_0z, &x[0]);
	float m_len = 1.0/sqrt(mag_0z[0] * mag_0z[0] + mag_0z[1] * mag_0z[1] + mag_0z[2] * mag_0z[2]);
	float a_len = 1.0/sqrt(acc_body[0]*acc_body[0] + acc_body[1]*acc_body[1] + acc_body[2]*acc_body[2]);

	// GNSS: position and velocity, plus vertical velocity
	if (use_gps)
	{
		add_observation(60.0, pos_north, x1[7], 0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(60.0, pos_east,  x1[8], 0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(5.0, vel_north, x1[10], 0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(5.0, vel_east, x1[11],  0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(15.0, gps.climb_rate, x1[12], 0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
	}

	// still motion, acc and gyro bias with very low noise.
	if (still)
	{
		add_observation(1e-3, acc_body[0]*a_len, -r[6], 2*x[2], 2*-x[3], 2*x[0], 2*-x[1], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-3, acc_body[1]*a_len, -r[7], -2*x[1], 2*-x[0], 2*-x[3], 2*-x[2], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-3, acc_body[2]*a_len, -r[8], -2*x[0], 2*x[1], 2*x[2], -2*x[3], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-4, -gyro[0], x1[4], 0.0,0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-4, -gyro[1], x1[5], 0.0,0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-4, -gyro[2], x1[6], 0.0,0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
	}

	// add a attitude observation if no assisting available(flow or GNSS)
	if (!still_inited || (!use_gps && !use_flow && !still))
	{
		add_observation(1e-1, acc_body[0]*a_len, -r[6], 2*x[2], 2*-x[3], 2*x[0], 2*-x[1], 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-1, acc_body[1]*a_len, -r[7], -2*x[1], 2*-x[0], 2*-x[3], 2*-x[2], 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-1, acc_body[2]*a_len, -r[8], -2*x[0], 2*x[1], 2*x[2], -2*x[3], 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
	}

	// mag
	if (mag_healthy)
	{
		add_observation(1e-1, mag_0z[0]*m_len, r[0], 2*x[0], 2*x[1], -2*x[2], -2*x[3], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-1, mag_0z[1]*m_len, r[1], -2*x[3], 2*x[2], 2*x[1], -2*x[0], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(1e-1, mag_0z[2]*m_len, r[2], 2*x[2], 2*x[3], 2*x[0], 2*x[1], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);

	}
	else
	{
		add_observation(60.0, mag_0z[0]*m_len, r[0], 2*x[0], 2*x[1], -2*x[2], -2*x[3], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(60.0, mag_0z[1]*m_len, r[1], -2*x[3], 2*x[2], 2*x[1], -2*x[0], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
		add_observation(60.0, mag_0z[2]*m_len, r[2], 2*x[2], 2*x[3], 2*x[0], 2*x[1], 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
	}

	// correction
	matrix R = matrix::diag(R_count, R_diag);
	matrix Hx = H * x;

	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();

	matrix residual = (zk - predicted_observation);
	matrix Kres = K*residual;

	x = x1 + K*(zk - predicted_observation);
	P = (matrix(P1.m) - K*H) * P1;

	// renorm
	float sqq = 1.0f/sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
	x[0] *= sqq;
	x[1] *= sqq;
	x[2] *= sqq;
	x[3] *= sqq;

	if (x[0] < 0)
	{
		x[0] = -x[0];
		x[1] = -x[1];
		x[2] = -x[2];
		x[3] = -x[3];
	}

	return 0;
}

void EKFINS::add_observation(float variance, float observation, float predicted, ...)	// ...: colomn of observation matrix
{
	R_diag[R_count++] = variance;
	zk[zk.m++] = observation;
	predicted_observation[predicted_observation.m++] = predicted;

	int dimension = H.n;
	int start = dimension*H.m;

	va_list vl;
	va_start(vl,predicted);
	for(int i=0; i<dimension; i++)
		H[start + i] = va_arg(vl,double);
	va_end(vl);

	H.m ++;
}

void EKFINS::remove_mag_ned_z(float *mag_body, float *q)
{
	float BODY2NED[3][3];
	float NED2BODY[3][3];

	float q0 = q[0];
	float q1 = q[1];
	float q2 = q[2];
	float q3 = q[3];

	NED2BODY[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;// 11
	NED2BODY[0][1] = 2.f * (q1*q2 + q0*q3);	// 12
	NED2BODY[0][2] = 2.f * (q1*q3 - q0*q2);	// 13
	NED2BODY[1][0] = 2.f * (q1*q2 - q0*q3);	// 21
	NED2BODY[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;// 22
	NED2BODY[1][2] = 2.f * (q2*q3 + q0*q1);	// 23
	NED2BODY[2][0] = 2.f * (q1*q3 + q0*q2);	// 31
	NED2BODY[2][1] = 2.f * (q2*q3 - q0*q1);	// 32
	NED2BODY[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;// 33

	BODY2NED[0][0] = NED2BODY[0][0];
	BODY2NED[0][1] = NED2BODY[1][0];
	BODY2NED[0][2] = NED2BODY[2][0];

	BODY2NED[1][0] = NED2BODY[0][1];
	BODY2NED[1][1] = NED2BODY[1][1];
	BODY2NED[1][2] = NED2BODY[2][1];

	BODY2NED[2][0] = NED2BODY[0][2];
	BODY2NED[2][1] = NED2BODY[1][2];
	BODY2NED[2][2] = NED2BODY[2][2];

	// transform vector from body frame to NED frame, discard z component
	float x = BODY2NED[0][0] * mag_body[0] + BODY2NED[0][1] * mag_body[1] + BODY2NED[0][2] * mag_body[2];
	float y = BODY2NED[1][0] * mag_body[0] + BODY2NED[1][1] * mag_body[1] + BODY2NED[1][2] * mag_body[2];

	// transform back to body frame
	mag_body[0] = NED2BODY[0][0] * x + NED2BODY[0][1] * y; // + NED2BODY[0][2] * 0;
	mag_body[1] = NED2BODY[1][0] * x + NED2BODY[1][1] * y; // + NED2BODY[1][2] * 0;
	mag_body[2] = NED2BODY[2][0] * x + NED2BODY[2][1] * y; // + NED2BODY[2][2] * 0;
}

int EKFINS::init_attitude(const float a[3], const float g[3], const float m[3])
{
	inited = true;

	float roll = atan2(-a[1], -a[2]) * 180 / 3.14159;
	float pitch = atan2(a[0], (-a[2] > 0 ? 1 : -1) * sqrt(a[1]*a[1] + a[2]*a[2])) * 180 / 3.14159;

	float cosRoll = cosf(roll);
	float sinRoll = sinf(roll);
	float cosPitch = cosf(pitch);
	float sinPitch = sinf(pitch);

	float magX = m[0] * cosPitch + m[1] * sinRoll * sinPitch + m[2] * cosRoll * sinPitch;
	float magY = m[1] * cosRoll - m[2] * sinRoll;
	float initialHdg = atan2f(-magY, magX);
	float cosHeading = cosf(initialHdg * 0.5f);
	float sinHeading = sinf(initialHdg * 0.5f);

	x[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	x[1] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	x[2] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	x[3] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	x[4] = g[0];
	x[5] = g[1];
	x[6] = g[2];

	for(int i=7; i<19; i++)
		x[i] = 0;

	return 0;
}

int EKFINS::get_euler(float *euler)
{

	Quaternion2RPY(&x[0], euler);

	return 0;
}