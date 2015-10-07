#pragma once

// Variable declaration
extern float q0, q1, q2, q3;		// quaternion of sensor frame relative to auxiliary frame
extern float gyro_bias[3];			// estimated gyro bias
extern float Rot_matrix[9];
extern float euler[3];
extern float NED2BODY[3][3];
extern float BODY2NED[3][3];
extern float acc_ned[3];			// acceleration rotated to north-east-down frame
extern float acc_horizontal[3];		// acceleration rotated to horizontal body frame, [0] points forward, [1] points right
extern float halfvx, halfvy, halfvz;// estimated gravity vector in body frame
extern float raw_yaw;				// raw mag yaw with tilt compensation
extern bool mag_ok;
extern float err_a[3];
extern float err_m[3];

// Function declarations
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz);
void NonlinearSO3AHRSupdate(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz, float twoKp, float twoKi, float twoKpMag, float twoKiMag, float dt);
