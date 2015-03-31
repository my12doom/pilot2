// a very simple AHRS module exported from MultiWii.
// there is no gyro bias estimation in this code, you need to do gyro bias compensating/estimating outside this module.
// recommended frame of reference:
// 

#pragma once

#include <utils/vector.h>

extern float roll;
extern float pitch;
extern float yaw_mag;
extern float yaw_gyro;
extern float accelz_mwc;

extern vector accel_ef;
extern vector estAccGyro;			// for roll & pitch
extern vector estMagGyro;			// for yaw
extern vector estGyro;				// for gyro only yaw, yaw lock on this

int ahrs_mwc_init(vector gyro_bias, vector accel, vector mag);
int ahrs_mwc_update(vector gyro, vector accel, vector mag, float dt);