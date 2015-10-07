#pragma once

#include "motion_detector.h"

class NonlinearSO3AHRS
{
public:
	NonlinearSO3AHRS();
	~NonlinearSO3AHRS();
	void update(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz, float twoKp, float twoKi, float twoKpMag, float twoKiMag, float dt);
	void update_gps(float vned[3], float dt);
	void reset();

//private:
	float q0, q1, q2, q3;		// quaternion of sensor frame relative to auxiliary frame
	float gyro_bias[3];			// estimated gyro bias
	float euler[3];
	float NED2BODY[3][3];
	float BODY2NED[3][3];
	float acc_ned[3];			// acceleration rotated to north-east-down frame
	float acc_horizontal[3];	// acceleration rotated to horizontal body frame, [0] points forward, [1] points right
	float est_acc[3];			// estimated gravity vector in body frame	
	float mag_avg[3];
	int mag_avg_count;

	float raw_yaw;				// raw mag yaw with tilt compensation
	bool mag_ok;
	float err_a[3];
	float err_m[3];

	motion_detector detect_acc;			// acc motion detecotr for initialization
	motion_detector detect_gyro;		// gyro motion detecotr for initialization
	bool initialized;

private:
};
