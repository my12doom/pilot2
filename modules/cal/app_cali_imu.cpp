#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "motion_detector.h"
#include "quaternion.h"
#include "app_cali_imu.hpp"
#include <HAL/Interface/ISysTimer.h>

// constants
static const float PI = M_PI;
float threshold_attitude = 20 * PI / 180;
float threshold_acc = 1;
float threshold_acc_attitude = 3;
float threshold_gyro = 5 * PI / 180;
int avg_needed = 6000;
int avg_progress_start = 250;
bool printf_en = false;

#define fit_func ellipsoid_fit
#define POS_COUNT 6
#define G_MS2 9.80665f

float ref_acc[6*3] = 
{
	0,0,G_MS2,
	0,0,-G_MS2,
	0,G_MS2,0,
	0,-G_MS2,0,
	G_MS2,0,0,
	-G_MS2,0,0,
};

float att_data[POS_COUNT][MAX_ACC_COUNT][6] = {0};
int g_avg_count[POS_COUNT];

int att_match = -1;
motion_detector mdet[MAX_ACC_COUNT];
int acc_count = 2;
bool finished = false;

// sensor fetching
bool worker_run = false;
int sock = -1;

// functions
int sphere_fit(const float *vec3_data, int count, float *vec3_center, float*scale, float *residual_avg, float *residual_max);
int ellipsoid_fit(const float *vec3_data, int count, float *vec3_center, float*scale, float *residual_avg, float *residual_max);
int calc_residual(const float *vec3_data, int count, float *vec3_center, float*scale, float *residual_avg, float *residual_max);
int pull_sensor(int sock, float q[4], float gyro0[3], float gyro1[3], float acc0[3], float acc1[3]);

// api implementation
int imu_cal_init(int acc_count /*= 2*/)
{
	if (acc_count > MAX_ACC_COUNT)
		return -1;

	// init/reste values
	memset(att_data, 0, sizeof(att_data));
	worker_run = true;
	att_match = -1;
	finished = false;
	::acc_count = acc_count;

	for(int i=0; i<MAX_ACC_COUNT; i++)
		mdet[i].set_threshold(threshold_acc);
	
	return 0;
}

int imu_cal_feed(float *acc, float *temperature, const char**acc_name /*= NULL*/)
{
	if (finished)
		return 0;
	
	int avg_count = 0;
	att_match = -1;
	for(int i=0; i<POS_COUNT; i++)
	{
		float dx = acc[0] - ref_acc[i*3+0];
		float dy = acc[1] - ref_acc[i*3+1];
		float dz = acc[2] - ref_acc[i*3+2];
		float diff = sqrt(dx*dx+dy*dy+dz*dz);
		if (diff < threshold_acc_attitude)
			att_match = i;
	}

	if (att_match >= 0)
	{
		bool still = true;
		for(int i=0; i<acc_count; i++)
			still &= !mdet[i].new_data(acc+3*i);

		avg_count = mdet[0].get_average(NULL);
		for(int i=1; i<acc_count; i++)
		{
			int count = mdet[i].get_average(NULL);
			if (count < avg_count)
				avg_count = count;
		}

		if (avg_count > avg_needed && avg_count > g_avg_count[att_match])
		{
			for(int i=0; i<acc_count; i++)
				mdet[i].get_average(att_data[att_match][i], att_data[att_match][i]+3);

			g_avg_count[att_match] = avg_count;
		}			
	}
	else
	{
		for(int i=0; i<acc_count; i++)
			mdet[i].reset();
	}

	static int64_t last_print = 0;
	if (systimer->gettime() - last_print > 50000)
	{
		char tmp[10] = {0};
		for(int i=0; i<POS_COUNT; i++)
			if (g_avg_count[i] > avg_needed)
				tmp[i] = '1';
			else
				tmp[i] = '0';
		printf("\r%d,%s, %d   ", att_match, tmp, avg_count);
		last_print = systimer->gettime();
	}
	

	bool all_done = true;
	for(int i=0; i<POS_COUNT; i++)
		if (g_avg_count[i] < avg_needed)
			all_done = false;

	if (all_done)
	{
		finished = true;
		printf("\ncollecting done\n");
		for(int i=0; i<acc_count; i++)
			printf("acc%d\t\t\t", i);
		printf("\n");

		for(int i=0; i<POS_COUNT; i++)
		{
			for(int j=0; j<acc_count; j++)
				printf("%.3f,%.3f,%.3f(%.4f,%.4f,%.4f)\t", att_data[i][j][0], att_data[i][j][1], att_data[i][j][2],
					att_data[i][j][3], att_data[i][j][4], att_data[i][j][5]);

			printf("\n");
		}

		for(int j=0; j<acc_count; j++)
		{
			float data[POS_COUNT*3];
			float center[3];
			float scale[3];
			float residual_avg, residual_max;
			for(int i=0; i<POS_COUNT; i++)
				memcpy(&data[3*i], att_data[i][j], 3*sizeof(float));
			
			fit_func(data, POS_COUNT, center, scale, &residual_avg, &residual_max);

			printf("acc%d(%s): bias=%.2f,%.2f,%.2f, scale=%.3f,%.3f,%.3f, temperature=%.2fC, residual(avg/max):%.5f/%.5fm/s2\n", j, (acc_name && acc_name[j])?acc_name[j]:"", center[0], center[1], center[2], 
				scale[0], scale[1], scale[2], temperature[j], residual_avg, residual_max);
		}


		printf("\n\n\n");

		return 0;
	}
	
	return 0;
}

// a & b : -PI ~ PI
// return a - b
static float radian_sub(float a, float b)
{
	float v1 = a-b;
	float v2 = a+2*PI-b;
	float v3 = a-2*PI-b;
	
	v1 = fabs(v1)>fabs(v2) ? v2 : v1;
	return fabs(v1)>fabs(v3) ? v3 : v1;
}

float attitude_diff(float rpy[2], float q[4])
{
	float q2[4];
	float rpy2[3] = {rpy[0], rpy[1], 0};
	RPY2Quaternion(rpy2, q2);

	quat_inverse(q2);

	float q_diff[4];
	quat_mult(q, q2, q_diff);

	return 2*asin(sqrt(q_diff[1]*q_diff[1]+q_diff[2]*q_diff[2]/*+q_diff[3]*q_diff[3]*/));
}

int sphere_fit(const float *vec3_data, int count, float *vec3_center, float*scale, float *residual_avg, float *residual_max)
{
	if (!vec3_data || !vec3_center || !scale)
		return -1;

	float mx1 = 0;
	float mx2 = 0;
	float mx3 = 0;

	float A0[9] = {0};
	float B0[3] = {0};

	for(int i=0; i<count*3; i+=3)
	{
		mx1 += vec3_data[i+0];
		mx2 += vec3_data[i+1];
		mx3 += vec3_data[i+2];
	}

	mx1 /= count;
	mx2 /= count;
	mx3 /= count;

	for(int i=0; i<count*3; i+=3)
	{
		float x1 = vec3_data[i+0];
		float x2 = vec3_data[i+1];
		float x3 = vec3_data[i+2];

		A0[0] += x1*(x1 - mx1);
		A0[1] += 2*(x1*(x2 - mx2));
		A0[2] += 2*(x1*(x3 - mx3));
		A0[4] += x2*(x2 - mx2);
		A0[5] += 2*(x2*(x3 - mx3));
		A0[8] += x3*(x3 - mx3);

		B0[0] += (x1*x1 +x2*x2 +x3*x3)*(x1 - mx1);
		B0[1] += (x1*x1 +x2*x2 +x3*x3)*(x2 - mx2);
		B0[2] += (x1*x1 +x2*x2 +x3*x3)*(x3 - mx3);
	}

	A0[0] /= count;
	A0[1] /= count;
	A0[2] /= count;
	A0[4] /= count;
	A0[5] /= count;
	A0[8] /= count;

	B0[0] /= count;
	B0[1] /= count;
	B0[2] /= count;

	matrix A(3, 3, A0);
	A = A + A.transpos();

	matrix B(1, 3, B0);

	matrix center = B * A.inverse();

	float radius = 0;
	for(int i=0; i<count*3; i+=3)
	{
		float dx1 = vec3_data[i+0] - center[0];
		float dx2 = vec3_data[i+1] - center[1];
		float dx3 = vec3_data[i+2] - center[2];
		radius += dx1*dx1+dx2*dx2+dx3*dx3;
	}

	vec3_center[0] = center[0];
	vec3_center[1] = center[1];
	vec3_center[2] = center[2];

	for(int i=0; i<3; i++)
		scale[i] =  G_MS2 / sqrt(radius/count);

	calc_residual(vec3_data, count, vec3_center, scale, residual_avg, residual_max);

	return 0;
}

int ellipsoid_fit(const float *vec3_data, int count, float *vec3_center, float*scale, float *residual_avg, float *residual_max)
{
	if (!vec3_data || !vec3_center || !scale)
		return -1;

	float m1[36] = {0};
	float m2[6] = {0};
	for(int i=0; i<count; i++)
	{
		// formula:
		// p[0]x + p[1]y + p[2]z - p[3]y^2 - p[4]z^2 + p[5] = x^2

		// Ht * H
		float x = vec3_data[3*i+0];
		float y = vec3_data[3*i+1];
		float z = vec3_data[3*i+2];
		float x2 = x*x;

		float v_tmp[6] = {x, y, z, -y*y, -z*z, 1};
		for(int j=0; j<6; j++)
			for(int k=0; k<6; k++)
				m1[j*6+k] += v_tmp[j] * v_tmp[k];

		// Ht * y
		m2[0] += x2*x;
		m2[1] += x2*y;
		m2[2] += x2*z;
		m2[3] += -y*y*x2;
		m2[4] += -z*z*x2;
		m2[5] += x2;
	}

	matrix HtH(6, 6, m1);
	matrix Hty(6, 1, m2);

	matrix X = HtH.inversef() * Hty;

	float x0 = X[0] / 2;
	float y0 = X[1] / (2*X[3]);
	float z0 = X[2] / (2*X[4]);

	float A = X[5] + x0*x0 + X[3]*y0*y0 + X[4] * z0*z0;
	float B = A/X[3];
	float C = A/X[4];

	float a = sqrt(A);
	float b = sqrt(B);
	float c = sqrt(C);

	vec3_center[0] = x0;
	vec3_center[1] = y0;
	vec3_center[2] = z0;

	scale[0] = G_MS2 / a;
	scale[1] = G_MS2 / b;
	scale[2] = G_MS2 / c;

	calc_residual(vec3_data, count, vec3_center, scale, residual_avg, residual_max);

	return 0;
}

static inline float fmax(float a, float b)
{
	return a>b?a:b;
}

int calc_residual(const float *vec3_data, int count, float *vec3_center, float*scale, float *residual_avg, float *residual_max)
{
	if (residual_avg)
	{
		*residual_avg = 0;
		if (residual_max)
			*residual_max = 0;

		for(int i=0; i<count*3; i+=3)
		{
			float dx1 = (vec3_data[i+0] - vec3_center[0]) * scale[0];
			float dx2 = (vec3_data[i+1] - vec3_center[1]) * scale[1];
			float dx3 = (vec3_data[i+2] - vec3_center[2]) * scale[2];

			float residual = fabs(sqrt(dx1*dx1+dx2*dx2+dx3*dx3) - G_MS2);
			if (residual_max)
				*residual_max = fmax(residual, *residual_max);

			*residual_avg += residual;
		}

		*residual_avg /= count;
	}

	return 0;
}

