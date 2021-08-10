#include "mag_calibration.h"
#include <Protocol/common.h>
#include <Protocol/RFData.h>
#include <utils/gauss_newton.h>
#include <utils/log.h>
#include <math.h>
#include <HAL/Resources.h>
#include <math/matrix.h>

#if !defined (__ARMCC_VERSION) || (__ARMCC_VERSION < 6010050)
static inline float fmax(float a, float b)
{
	return a>b?a:b;
}

static inline float fmin(float a, float b)
{
	return a<b?a:b;
}
#endif

mag_calibration::mag_calibration()
{
	reset();
}

mag_calibration::~mag_calibration()
{
}

// reset all
int mag_calibration::reset()
{
	calibration_error_code = -1;
	stage = stage_horizontal;
	count = 0;
	rotation = 0;
	
	return 0;
}

int mag_calibration::add_data(float *newdata)
{
	if (count >= MAX_DATA_COUNT)
		return -1;
	
	// check for near points
	for(int i=count-1; i>=0; i--)
	{
		float dx = newdata[0] - data[i*3+0];
		float dy = newdata[1] - data[i*3+1];
		float dz = newdata[2] - data[i*3+2];
		float distance = dx*dx + dy*dy + dz*dz;
		if (distance < MIN_DISTANCE_SQ2)
			return -2;
	}

	data[3*count+0] = newdata[0];
	data[3*count+1] = newdata[1];
	data[3*count+2] = newdata[2];

	count++;

	return 0;
}

// provide realtime data for the calibrator to collect desired data.
int mag_calibration::provide_data(float *mag_data, float *euler, float *gyro, float dt)
{
	if (stage == stage_horizontal)
	{
		float cos_roll = cos(euler[0]);
		float cos_pitch = cos(euler[1]);
		float tilt = sqrt(cos_roll*cos_roll + cos_pitch*cos_pitch);

		// are we flat?
		if (cos_roll>0 && cos_pitch>0 && tilt > cos(MAX_TILT*PI/180))
		{
			rotation += gyro[2] * dt;

			add_data(mag_data);
		}

		// enough rotation, proceed to vertical, nose/tail down
		if (fabs(rotation) > 2.0f * PI)
		{
			LOGE("GO VERTICAL PITCH\n");
			rotation = 0;
			stage = stage_vertical_pitch;
		}
	}

	if (stage == stage_vertical_pitch)
	{
		// nose/tail down?
		if (fabs(-PI/2 - euler[1]) < MAX_TILT*PI/180)
		{
			rotation += gyro[0] * dt;

			add_data(mag_data);
		}

		// enough rotation, proceed to vertical, wing down
		if (fabs(rotation) > 2.0f * PI)
		{
			rotation = 0;
			LOGE("GO VERTICAL ROLL\n");
			stage = stage_ready_to_calibrate;//stage_vertical_roll;
		}
	}
	
	if (stage == stage_vertical_roll)
	{
		// wing down?
		if (fabs(-PI/2 - euler[0]) < MAX_TILT*PI/180)
		{
			rotation += gyro[1] * dt;

			add_data(mag_data);
		}

		// enough rotation, we are ready
		if (fabs(rotation) > 2.0f * PI)
		{
			stage = stage_ready_to_calibrate;
		}
	}


	return 0;
}

// get calibration stage for user interaction
// for horizontal stage, user need to hold the airframe horizontally, and spin around z axis, pointing airframe heads to each geographic direction
// for horizontal stage, user need to hold the airframe vertically heads down, and spin around x axis, pointing airframe upside to each geographic direction
mag_calibration_stage mag_calibration::get_stage()
{
	return stage;
}

bool invalid_float(float v)
{
	return isnan(v) || !isfinite(v);
}

// do the calibration, this will take longger time, don't call this in main loop or any time-criticial functions.
// you may call this in a lower priority timer.
// return value:
// -1 immediatly if not enough data collected or calibrating.
// -2 if collection completed but very little data collected, this usually indicate failure in compass hardware
// other values are same as get_result()
int mag_calibration::do_calibration()
{
	result.num_points_collected = count;
	if (stage == stage_data_calibrated)
		return 0;
	if (stage != stage_ready_to_calibrate)
		return -1;
	if (count < MIN_DATA_COUNT)
	{
		stage = stage_data_calibrated;
		calibration_error_code = -2;
		return -2;
	}

	// do logging!
	if (log_ready())
	for(int i=0; i<count; i++)
	{
		mag_collecting_data d = 
		{
			{data[i*3]*10, data[i*3+1]*10, data[i*3+2]*10,},
			i == count-1,
		};

		int64_t timestamp = systimer->gettime();
		int res;
		do
		{
			res = log(&d, TAG_MAG_COLLECTING_DATA, timestamp);
		}
		while(res != 0 && log_ready());
	}

	stage = stage_calibrating;

	gauss_newton_sphere_fitting fitter;
	fitter.calculate(data, count);
	fitter.get_result(result.bias);

	bool used_2nd_algorithm = false;
retry:
	// statistics
	result.residual_average = 0;
	result.residual_min = 1e+9;
	result.residual_max = -1e+9;
	for(int i=0; i<count; i++)
	{
		float x = (data[3*i+0] + result.bias[0]) * result.scale[0];
		float y = (data[3*i+1] + result.bias[1]) * result.scale[1];
		float z = (data[3*i+2] + result.bias[2]) * result.scale[2];
		float residual = fabs(1 - sqrt(x*x+y*y+z*z));
		result.residual_average += residual;
		result.residual_max = fmax(result.residual_max, residual);
		result.residual_min = fmin(result.residual_min, residual);
	}
	result.residual_average /= count;

	// normalize scale factor
	result.scale[0] *= NORM_SCALE;
	result.scale[1] *= NORM_SCALE;
	result.scale[2] *= NORM_SCALE;
	
	// check for possible failure
	calibration_error_code = 0;
	if (result.residual_average > MAX_AVG_RESIDUAL || result.residual_max > MAX_MAX_RESIDUAL)
	{
		calibration_error_code = 1;
	}

	else
	{
		for(int i=0; i<3; i++)
		{
			if (fabs(result.bias[i]) > MAX_OFFSET || result.scale[i] < MIN_SCALE || result.scale[i] > MAX_SCALE)
			{
				calibration_error_code = 2;
			}

			if (invalid_float(result.bias[i]) || invalid_float(result.scale[i]))
			{
				calibration_error_code = 2;
			}
		}
	}

	// try 2nd algorithm if gauss-newton failed
	if (!used_2nd_algorithm && calibration_error_code != 0)
	{
		used_2nd_algorithm = true;
		calibration2();
		goto retry;
	}

	stage = stage_data_calibrated;
	
	return calibration_error_code;
}

int mag_calibration::calibration2()
{
	LOGE("using mag calibration algorithm 2\n");

	float m1[36] = {0};
	float m2[6] = {0};
	for(int i=0; i<count; i++)
	{
		// formula:
		// p[0]x + p[1]y + p[2]z - p[3]y^2 - p[4]z^2 + p[5] = x^2

		// Ht * H
		float x = data[3*i+0];
		float y = data[3*i+1];
		float z = data[3*i+2];
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

	result.bias[0] = -x0;
	result.bias[1] = -y0;
	result.bias[2] = -z0;

	result.scale[0] = 1 / a;
	result.scale[1] = 1 / b;
	result.scale[2] = 1 / c;

	return 0;
}

// get calibration result
// return value:
//  0: everything fine.
//	-1 for not enough data collected or calibrating.
//  negtive value for other criticial error.
//  1 for possible large residual or large magnetic interference, threshold values are defined in beginning of header file.
//  2 for very large offset or scale factor, this usually indicate a sensor error.
int mag_calibration::get_result(mag_calibration_result *result)
{
	if (stage == stage_data_calibrated)
		*result = this->result;

	return calibration_error_code;
}
