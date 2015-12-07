#pragma once

#include <stdint.h>
#include <math.h>

//EKF Input varible EKF_U
typedef struct{
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float accel_x;
	float accel_y;
	float accel_z;
}EKF_U;

//EKF Mesurement varible EKF_Mesurement
typedef struct{
	float Mag_x;
	float Mag_y;
	float Mag_z;
	float Pos_GPS_x;
	float Pos_GPS_y;
	float Pos_Baro_z;//we don't trust gps altitude
	float Vel_GPS_x;
	float Vel_GPS_y;
	//no Vel_GPS_z
	/* Optical Flow */
	float Pos_FLOW_x;
	float Pos_FLOW_y;
	float Pos_FLOW_z;
	float Vel_FLOW_x;
	float Vel_FLOW_y;
	float Vel_FLOW_z;
}EKF_Mesurement;

//EKF Estimator Result X (NED frame)
typedef struct{
	float Pos_x;
	float Pos_y;
	float Pos_z;
	float Vel_x;
	float Vel_y;
	float Vel_z;
	float roll;
	float pitch;
	float yaw;
	float q0,q1,q2,q3;
}EKF_Result;


class ekf_estimator
{
private:
	bool ekf_is_init;
	float gyro_bias[3];//store gyro zero bias
	float Be[3],P[169],X[13],Q[81],R[64];
	//This is for check ekf init
	float ekf_loopcount;
	float que_window[3][50];
	
public:


	EKF_Result ekf_result;

	ekf_estimator();
	~ekf_estimator();

	void init(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz);
	void init(float roll,float pitch,float yaw);

	int update(EKF_U u,EKF_Mesurement mesurement,const float dT);
	bool ekf_is_ready();

	//Mag ground length
	float ground_mag_length;
	float ekf_is_convergence;
	float sqrt_variance[3];
	


};
