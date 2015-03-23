#include "altitude_estimator.h"
#include <string.h>
#include "../common/common.h"

static float P[16] = 
{
	200, 0, 0, 0,
	0, 200, 0, 0,
	0, 0, 200, 0,
	0, 0, 0, 200
};	// 4x4 matrix, covariance
static float Q[16] = 
{
	4e-6, 0, 0, 0,
	0, 1e-6, 0, 0,
	0, 0, 1e-6, 0,
	0, 0, 0, 1e-7,
};
static float R[4] = 
{
	80, 0,
	0, 0.0063,
};

static float R2[4] = 
{
	600, 0,
	0, 0.00230,
};


altitude_estimator::altitude_estimator()
:compensate_land_effect(false)
{
	state[0] = 0;
	state[1] = 0;
	state[2] = 0;
	state[3] = 0;
}

altitude_estimator::~altitude_estimator()
{

}

void altitude_estimator::matrix_error(const char*msg)
{
	LOGE(msg);
	while(true)
		;
}

void altitude_estimator::matrix_mov(float *dst, const float *src, int row, int column)
{
	memcpy(dst, src, row*column*4);
}

void altitude_estimator::matrix_add(float *op1, float *op2, int row, int column)
{
	int count = row * column;
	for(int i=0; i<count; i++)
		op1[i] += op2[i];
}
void altitude_estimator::matrix_sub(float *op1, float *op2, int row, int column)
{
	int count = row * column;
	for(int i=0; i<count; i++)
		op1[i] -= op2[i];
}

int altitude_estimator::matrix_mul(float *out, const float *m1, int row1, int column1, const float *m2, int row2, int column2)
{
	if (column1 != row2)
		matrix_error("invalid matrix_mul");

	for(int x1 = 0; x1<column2; x1++)
	{
		for(int y1 = 0; y1<row1; y1++)
		{
			out[y1*column2+x1] = 0;
			for(int k = 0; k<column1; k++)
				out[y1*column2+x1] += m1[y1*column1+k] * m2[k*column2+x1];
		}
	}

	return 0;
}

int altitude_estimator::inverse_matrix2x2(float *m)
{
	float det = m[0] * m[3] - m[1] * m[2];
	if (det == 0)
		return -1;
	float t[4] = {m[0], m[1], m[2], m[3]};

	m[0] = t[3]/det;
	m[1] = -t[1]/det;
	m[2] = -t[2]/det;
	m[3] = t[0]/det;

	return 0;
}

int altitude_estimator::update(float accelz, float baro, float dt)
{
	if (dt > 0.2f)
		return -1;

	// near ground, reject ground effected baro data
// 	bool invalid_baro_data = mode == quadcopter && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(state[0] - ground_altitude) < 1.0f);
	bool invalid_baro_data = isnan(baro);


	float dtsq = dt*dt;
	float dtsq2 = dtsq/2;
	float F[16] = 
	{
		1, dt, dtsq2, dtsq2,
		0, 1, dt, dt,
		0, 0, 1, 0,
		0, 0, 0, 1,
	};
	float FT[16] = 
	{
		1, 0, 0, 0,
		dt, 1, 0, 0,
		dtsq2, dt, 1, 0,
		dtsq2, dt, 0, 1,
	};

	static float Hab[2*4] = 
	{
		1, 0, 0, 0,
		0, 0, 1, 0,
	};
	static float HabT[4*2] = 
	{
		1, 0,
		0, 0,
		0, 1,
		0, 0,
	};

	// accelerometer only
	static float Ha[2*4] = 
	{
		0, 0, 1, 0,
	};
	static float HaT[4*2] = 
	{
		0,
		0,
		1,
		0,
	};

	float zk_ab[2] = {baro, accelz};
	float zk_a[2] = {accelz};

	float *H = invalid_baro_data ? Ha : Hab;
	float *HT = invalid_baro_data ? HaT : HabT;
	float *zk = invalid_baro_data ? zk_a : zk_ab;
	int observation_count = invalid_baro_data ? 1 : 2;

	float state1[4];
	float P1[16];
	float tmp[16];
	float tmp2[16];
	float tmp3[16];
	float kg[8];


	// predict
	matrix_mul(state1, F, 4, 4, state, 4, 1);
	matrix_mul(tmp, P, 4, 4, FT, 4, 4);
	matrix_mul(P1, F, 4, 4, tmp, 4, 4);

	// covariance
	matrix_add(P1, Q, 4, 4);

	// controll vector
	//state1[2] = state1[2] * 0.8f * 0.2f * (target_accel);
	// 	if (invalid_baro_data)
	// 		state1[1] *= 0.995f;

	// update

	// kg
	matrix_mul(tmp, P1, 4, 4, HT, 4, observation_count);
	matrix_mul(tmp2, H, observation_count, 4, P1, 4, 4);
	matrix_mul(tmp3, tmp2, observation_count, 4, HT, 4, observation_count);
	if (observation_count == 2)
	{
		matrix_add(tmp3, compensate_land_effect ? R2 : R, observation_count, observation_count);
		inverse_matrix2x2(tmp3);
	}
	else
	{
		tmp3[0] += R[3];
		tmp3[0] = 1.0f / tmp3[0];
	}
	matrix_mul(kg, tmp, 4, observation_count, tmp3, observation_count, observation_count);


	// update state
	// residual
	matrix_mul(tmp, H, observation_count, 4, state1, 4, 1);
	matrix_sub(zk, tmp, observation_count, 1);

	matrix_mul(tmp, kg, 4, observation_count, zk, observation_count, 1);
	matrix_mov(state, state1, 4, 1);
	matrix_add(state, tmp, 4, 1);

	// update P
	float I[16] = 
	{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	};
	matrix_mul(tmp, kg, 4, observation_count, H, observation_count, 4);
	matrix_sub(I, tmp, 4, 4);
	matrix_mul(P, I, 4, 4, P1, 4, 4);

	TRACE("\rtime=%.3f,state:%.2f,%.2f,%.2f,%.2f, raw:%.3f, accelz:%.3f      ", getus()/1000000.0f, state[0], state[1], state[2], state[3], baro, accelz);

	TRACE("pressure=%.2f\r", a_raw_pressure);

	return 0;
}
