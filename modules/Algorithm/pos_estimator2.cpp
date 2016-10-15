#include "pos_estimator2.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#define sonar_step_threshold 0.45f

const float attitude_error_yaw = sin(10*PI/180);
const float attitude_error_roll_pitch = sin(1*PI/180);
const float gain_error = 0.003;
const float mis_alignment_error = 0.02;

#ifdef WIN32
float baro_comp_coeff[6] = 
{
	0.18, -0.08,
	0.25, -0.25,
	-0.5, -0,
};
#else
#include <utils/param.h>
param baro_comp_coeff[6] = 
{
	param("bcxp", 0.18f), param("bcxn", -0.08f),
	param("bcyp", 0.25f), param("bcyn", -0.25f),
	param("bczp", -0.50f), param("bczn", -0.00f),
};
#endif

pos_estimator2::pos_estimator2()
{
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
	ticker = 0;
	sonar_ticker = 0;
	sonar_healthy = false;
	position_healthy = false;
	flow_healthy = false;
	last_valid_sonar = 0;
	memset(local, 0, sizeof(local));
	flow_ticker = 0;
	latency = 400000;
	gps_north = 0;
	gps_east = 0;
	_state = 1;
	vx_lpf = 0;
	vy_lpf = 0;
	saturation_timeout = 0.3f;

	P = matrix::diag(13, 100.0, 100.0, 4.0, 100.0, 100.0, 4.0, 1.0, 1.0, 1.0, 100.0, 100.0, 100.0, 10.0);
	x = matrix(13,1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	Q = matrix::diag(13, 4e-6, 4e-6, 4e-6, 
						5e-6, 5e-6, 5e-6, 
						1e-7, 1e-7, 1e-7, 
						1e-7, 1e-7, 5e-7, 4e-6);

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


int pos_estimator2::update(const float q[4], const float acc_body[3], devices::gps_data gps, float baro, float dt, bool armed, bool airborne)			// unit: meter/s
{
	// 	DCM
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


	Q(3,3) = armed ? 5e-4 : 1e-3;
	Q(4,4) = armed ? 5e-4 : 1e-3;
	Q(5,5) = armed ? 1e-6 : 1e-3;

	// flow switching
	bool use_flow = last_valid_sonar < 3.5f && (frame.ground_distance > 0) && (frame.qual > 133) && r[8] > 0.7f;		// note:0.7 ~= cos(30deg), use flow in less than 30degree flight
	if (!flow_healthy && use_flow)
	{
		flow_ticker += dt;

		if (flow_ticker > 0.3f)
		{
			LOGE("pos_estimator2: using flow\n");
			flow_healthy = true;
		}
	}
	else if (flow_healthy && !use_flow)
	{
		flow_ticker += dt;

		if (flow_ticker > 0.3f)
		{
			LOGE("pos_estimator2: flow failed\n");
			flow_healthy = false;
		}
	}
	else
	{
		flow_ticker = 0;
	}

	// sonar switching
	if (!sonar_healthy && frame.ground_distance > 0)
	{
		sonar_ticker += dt;

		if (sonar_ticker > 1)
		{
			LOGE("pos_estimator2: using sonar\n");
			sonar_healthy = true;
			P[12*14] = 10.0;
			x[12] = x.data[2] - frame.ground_distance/1000.0f;
		}
	}

	else if (sonar_healthy && frame.ground_distance <= 0)
	{
		sonar_ticker += dt;

		if (sonar_ticker > 1)
		{
			LOGE("pos_estimator2: sonar disabled\n");
			sonar_healthy = false;
		}

	}
	else
	{
		sonar_ticker = 0;
	}

	// GPS switching
	bool use_gps = (gps.fix == 3 && gps.position_accuracy_horizontal < 3.5f && gps.velocity_accuracy_horizontal < 0.8f && gps.DOP[1] < 150) || (position_healthy && gps.position_accuracy_horizontal < 7.0f && gps.velocity_accuracy_horizontal < 2.0f && gps.DOP[1] < 300);

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

		if (ticker > 1)
		{
			position_healthy = false;
			ticker = 0;

			LOGE("pos_estimator2: position failed\n");

			_state = 1;
		}
	}
	else
	{
		ticker = 0;
		_state = use_gps ? 3 : 1;
	}

	// prepare matrices and helper variables
	float yaw = atan2f(r[3], r[0]);
	float dtsq_2 = dt*dt/2;
	acc_ned[0] = r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2];
	acc_ned[1] = r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2];
	acc_ned[2] = -(r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2);

	acc_ned[0] += r[0]* x[6] + r[1] *x[7] + r[2] * x[8];
	acc_ned[1] += r[3]* x[6] + r[4] *x[7] + r[5] * x[8];
	acc_ned[2] += -(r[6]* x[6] + r[7] *x[7] + r[8] * x[8]);

	v_bf[0] = r[0]*x[3] + r[3]*x[4] + r[6]*x[5];
	v_bf[1] = r[1]*x[3] + r[4]*x[4] + r[7]*x[5];
	v_bf[2] = r[2]*x[3] + r[5]*x[4] + r[8]*x[5];

	float R_baro = 25.0f;
	if (position_healthy/* || flow_healthy*/)
	{
		baro_comp = 0;
		baro_comp += v_bf[0] > 0 ? (v_bf[0] * baro_comp_coeff[0]) : (v_bf[0] * baro_comp_coeff[1]);
		baro_comp += v_bf[1] > 0 ? (v_bf[1] * baro_comp_coeff[2]) : (v_bf[1] * baro_comp_coeff[3]);
		baro_comp += v_bf[2] > 0 ? (v_bf[2] * baro_comp_coeff[4]) : (v_bf[2] * baro_comp_coeff[5]);
		baro -= baro_comp;

		if (v_bf[2] > 1.0f)
			R_baro = 100.0f;
	}

	if(armed && !airborne)
		R_baro = 500;

	matrix F = matrix(13,13,
		1.0,0.0,0.0, dt, 0.0, 0.0, dtsq_2*r[0], dtsq_2*r[1], dtsq_2*r[2], 0.0, 0.0, 0.0, 0.0,
		0.0,1.0,0.0, 0.0, dt, 0.0, dtsq_2*r[3], dtsq_2*r[4], dtsq_2*r[5], 0.0, 0.0, 0.0, 0.0,
		0.0,0.0,1.0, 0.0, 0.0, dt, -dtsq_2*r[6], -dtsq_2*r[7], -dtsq_2*r[8], 0.0, 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 1.0,0.0,0.0, dt*r[0], dt*r[1], dt*r[2], 0.0, 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 0.0,1.0,0.0, dt*r[3], dt*r[4], dt*r[5], 0.0, 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,1.0, -dt*r[6], -dt*r[7], -dt*r[8], 0.0, 0.0, 0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0,
		0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 1.0
		);
	matrix Bu = matrix(13,1,
		dtsq_2 * (r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2]),
		dtsq_2 * (r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2]),
		-dtsq_2 * (r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2),
		dt * (r[0]* acc_body[0] + r[1] *acc_body[1] + r[2] * acc_body[2]),
		dt * (r[3]* acc_body[0] + r[4] *acc_body[1] + r[5] * acc_body[2]),
		-dt * (r[6]* acc_body[0] + r[7] *acc_body[1] + r[8] * acc_body[2] + G_in_ms2),
		0.0,0.0,0.0,
		0.0,0.0,0.0, 0.0
		);

#if 1
	matrix G = matrix(6, 3, 
		dtsq_2 * r[0], dtsq_2 *  r[1], dtsq_2 * r[2],
		dtsq_2 * r[3], dtsq_2 *  r[4], dtsq_2 * r[5],
		-dtsq_2 * r[6], dtsq_2 *  r[7], dtsq_2 * r[8],
		dt * r[0], dt *  r[1], dt * r[2],
		dt * r[3], dt *  r[4], dt * r[5],
		-dt * r[6], dt *  r[7], dt * r[8]
		);

	float acc_ned_abs[4] = {fabs(acc_ned[0]), fabs(acc_ned[1]), fabs(acc_ned[2]), fabs(acc_ned[0])+ fabs(acc_ned[1])+ fabs(acc_ned[2])};
	float acc_bf_abs[4] = {fabs(acc_body[0]), fabs(acc_body[1]), fabs(acc_body[2]), fabs(acc_body[0])+ fabs(acc_body[1])+ fabs(acc_body[2])};

// 	const float attitude_error_yaw = sin(10*PI/180);
// 	const float attitude_error_roll_pitch = sin(1*PI/180);
// 	const float gain_error = 0.003;
// 	const float mis_alignment_error = 0.02;

	float err0 = attitude_error_yaw * acc_bf_abs[1] + attitude_error_roll_pitch * acc_bf_abs[2] + gain_error * acc_bf_abs[0]  + mis_alignment_error * (acc_bf_abs[1]+acc_bf_abs[2]);
	float err1 = attitude_error_yaw * acc_bf_abs[0] + attitude_error_roll_pitch * acc_bf_abs[2] + gain_error * acc_bf_abs[1]  + mis_alignment_error * (acc_bf_abs[0]+acc_bf_abs[2]);
	float err2 =									  attitude_error_roll_pitch * (acc_bf_abs[0]+acc_bf_abs[1]) + gain_error * acc_bf_abs[2]  + mis_alignment_error * (acc_bf_abs[0]+acc_bf_abs[1]);

	matrix Q0 = matrix::diag(3, err0*5, err1*5, err2*5);		// covariance increment including noise, mis-alignment/attitude/gain error.
	matrix Q1 = G * Q0 * Q0 * G.transpos();
	for(int m=0; m<6; m++)
		for(int n=0; n<6; n++)
			Q(m,n) = Q1(m,n);

	co[0] = Q1(1,1)*1E10;

	//Q(2,2) += dt * sqrt(x[0]*x[0]+x[1]*x[1]) * 1E-2;

	if (!armed)
	{
// 		Q(3,3) = 1e-3;
// 		Q(4,4) = 1e-3;
// 		Q(5,5) = 1e-3;
	}


#endif

	matrix x1 = F * x + Bu;
	matrix P1 = F * P * F.transpos() + Q/* * dt / 0.05f*/;		// note: Q values are tuned for dt = 0.05f, so we normalize it to dt to achieve proper covariance growth.

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

	int R_count = 0;
	float R_diag[13];

	if (use_gps)
	{
		zk = matrix(6,1,pos_north, pos_east, baro, vel_north, vel_east, gps.climb_rate);
// 		R = matrix::diag(6, 60.0, 60.0, 60.0, 5.0, 5.0, 15.0);
		H = matrix(6,13,
			1.0, 0.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,
			0.0, 0.0, 0.0, 1.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0, 0.0,
			0.0, 0.0, 0.0, 0.0,1.0,0.0, 0.0,0.0,0.0, 0.0,1.0,0.0, 0.0,
			0.0, 0.0, 0.0, 0.0,0.0,1.0, 0.0,0.0,0.0, 0.0,0.0,1.0, 0.0);

		float R_pos = 4.0f * gps.position_accuracy_horizontal * gps.position_accuracy_horizontal;
		float R_vel = 4.0f * gps.velocity_accuracy_horizontal * gps.velocity_accuracy_horizontal;

// 		R_pos = 25;
// 		R_vel = 5;

		R_count = 6;
		R_diag[0] = R_pos;
		R_diag[1] = R_pos;
		R_diag[2] = R_baro;
		R_diag[3] = R_vel;
		R_diag[4] = R_vel;
		R_diag[5] = 65.0;


		if (gps.DOP[1] > 200)
		{
			printf("gps glitch\n");
		}

	}
	else
	{
		float pixel_compensated_x = frame.pixel_flow_x_sum;
		float pixel_compensated_y = frame.pixel_flow_y_sum;

		if (fabs(gyro[0]) > 60 * PI / 180 || fabs(gyro[1]) > 60 * PI / 180)
			saturation_timeout = 0.3f;

		saturation_timeout -= dt;
		bool saturation = saturation_timeout > 0;

		
 		if (/*fabs(pixel_compensated_x) > 5 && */fabs(pixel_compensated_x) < 35 /*&& fabs(pixel_compensated_y)>5 */&& fabs(pixel_compensated_y)<35 && !saturation)
 		{
			 pixel_compensated_x -= gyro[0] * 18000 / PI * 0.0028f;
			 pixel_compensated_y -= gyro[1] * 18000 / PI * 0.0028f;
 		}
		

		float wx = pixel_compensated_x / 28.0f * 100 * PI / 180;
		float wy = pixel_compensated_y / 28.0f * 100 * PI / 180;

		vx = wx * last_valid_sonar;
		vy = wy * last_valid_sonar;

		float alpha5 = dt / (dt + 1.0f/(2 * PI * 15.0f));

		if (!use_flow)
			vx = vy = 0;

		vx_lpf = vx * alpha5 + (1-alpha5) * vx_lpf;
		vy_lpf = vy * alpha5 + (1-alpha5) * vy_lpf;

		zk = matrix(3,1,baro, vx, vy, last_valid_sonar);
		H = use_flow ? 
			matrix(3,13,
			0.0,0.0,1.0, 0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0, 0.0,
			0.0,0.0,0.0, -r[1],-r[4],0.0,  0.0,0.0,0.0,  0.0,0.0,0.0, 0.0,
			0.0,0.0,0.0, r[0],r[3],0.0,  0.0,0.0,0.0,  0.0,0.0,0.0, 0.0				// strange flow coordinates
			) :
			matrix(3,13,
			0.0,0.0,1.0, 0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0, 0.0,
			0.0,0.0,0.0, 1.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0, 0.0,
			0.0,0.0,0.0, 0.0,1.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0, 0.0
			);
// 		R = matrix::diag(count,600.0, saturation?50.0 : 5.0, saturation ? 50.0 : 5.0, 16.0);

		float height_factor = last_valid_sonar;
		if (height_factor < 1.0f)
			height_factor = 1.0f;
		height_factor *= height_factor;

		R_count = 3;
		R_diag[0] = R_baro;
		R_diag[1] = (saturation?25.0f : 5.0f) * height_factor;
		R_diag[2] = (saturation?25.0f : 5.0f) * height_factor;
		R_diag[3] = 16.0f;

#ifdef WIN32
		if (use_flow)
		{
			matrix hx1 = H*x1;
			predict_flow[0] = hx1[1];
			predict_flow[1] = hx1[2];
		}
#endif
	}

	if (sonar_healthy)
	{

		if (frame.ground_distance > 0)
		{
			float new_sonar = frame.ground_distance/1000.0f;
			float predicted_sonar = x1[2] - x1[12];

			if (fabs(new_sonar-predicted_sonar) > sonar_step_threshold)
			{
				LOGE("sonar step response: %.2f->%.2f\n", predicted_sonar, new_sonar);
				x[12] = x[2] - new_sonar;
				x1[12] = x1[2] - new_sonar;
			}
			last_valid_sonar = new_sonar;

			float sonar_h[13] = {0,0,1.0f, 0,0,0, 0,0,0, 0,0,0,-1.0f};
			memcpy(H.data + H.m*H.n, sonar_h, H.n*4);
			H.m++;
			R_diag[R_count++] = 5.0f;
			zk[zk.m++] = last_valid_sonar;
		}
	}

	R = matrix::diag(R_count, R_diag);


	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();

	float cos_yaw = cos(yaw);
	float sin_yaw = sin(yaw);
	v_hbf[0] = cos_yaw * x[3] + sin_yaw * x[4];
	v_hbf[1] = -sin_yaw * x[3] + cos_yaw * x[4];
	// -->> vx ~= -v_hbf[1]
	// -->> vy ~= v_hbf[0]

	x = x1 + K*(zk - H*x1);
	P = (matrix(P1.m) - K*H) * P1;

	float alpha05 = dt / (dt + 1.0f/(2 * PI * 0.2f));

	local[0] = (local[0] + x[3] * dt) * (1-alpha05) + alpha05 * x[0];
	local[1] = (local[1] + x[4] * dt) * (1-alpha05) + alpha05 * x[1];

	// sanity check
	for(int i=0; i<x.m; i++)
		if (isnan(x[i] || !isfinite(x[i])) || x[i] != x[i])
		{
			reset();
			return -1;
		}

	return 0;
}
