#pragma once

#define MIN_DISTANCE_SQ2 100.0f
#define MAX_DATA_COUNT 500
#define MAX_OFFSET 500.0f
#define NORM_SCALE 500.0f			// normalized output magnetic field intensity in milli-gauss.
#define MAX_SCALE 2.5f				// http://en.wikipedia.org/wiki/Earth%27s_magnetic_field, The field ranges between approximately 25,000 and 65,000 nT (0.25¨C0.65 G)
#define MIN_SCALE 0.33f				// we accept 0.2 ~ 1.5 G due to soft-iron (100 - 1500 milli-gauss)
#define MAX_MAX_RESIDUAL 0.25f		// 25%, same as ahrs magnetic interference detection threshold
#define MAX_AVG_RESIDUAL 0.05f		// twice of HMC5983 non-linearity + cross-axis sensitivity.
#define MAX_TILT 20.0f				// 20 degreee max tilt

enum mag_calibration_stage
{
	stage_horizontal = 0,
	stage_vertical = 1,
	stage_ready_to_calibrate = 2,
	stage_data_calibrated = 3,
};

typedef struct
{
	float bias[3];
	float scale[3];
	float residual_average;
	float residual_max;
	float residual_min;
} mag_calibration_result;


class mag_calibration
{
public:
	mag_calibration();
	~mag_calibration();

	// reset all
	int reset();

	// provide realtime data for the calibrator to collect desired data.
	int provide_data(float *mag_data, float *euler, float *gyro, float dt);

	// get calibration stage for user interaction
	// for horizontal stage, user need to hold the airframe horizontally, and spin around z axis, pointing airframe heads to each geographic direction
	// for horizontal stage, user need to hold the airframe vertically heads down, and spin around x axis, pointing airframe upside to each geographic direction
	mag_calibration_stage get_stage();


	// do the calibration, this will take longger time, don't call this in main loop or any time-criticial functions.
	// you may call this in a lower priority timer.
	// return value:
	// -1 immediatly if not enough data collected.
	// other values are same as get_result()
	int do_calibration();


	// get calibration result
	// return value:
	//  0: everything fine.
	//	-1 for not enough data collected.
	//  negtive value for other criticial error.
	//  1 for possible large residual or large magnetic interference, threshold values are defined in beginning of header file.
	//  2 for very large offset or scale factor, this usually indicate a sensor error.
	int get_result(mag_calibration_result *result);

protected:

	int add_data(float *newdata);

	float data[MAX_DATA_COUNT*3];
	int count;
	mag_calibration_stage stage;
	mag_calibration_result result;
	int calibration_error_code;
	float rotation;
};