#include "ekf_estimator.h"
#include "init_quaternion_by_euler.h"
#include "init_ekf_matrix.h"
#include "INS_SetState.h"
#include "LinearFG.h"
#include "RungeKutta.h"
#include "INS_CovariancePrediction.h"
#include "INS_Correction.h"
#include "quaternion_to_euler.h"
ekf_estimator::ekf_estimator()
{
}
ekf_estimator::~ekf_estimator()
{
}
void ekf_estimator::init(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz)
{
	float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;
	float q0,q1,q2,q3;//quaternion
	ground_mag_length = sqrt(mx*mx + my*my + mz*mz);


	gyro_bias[0] = gx;
	gyro_bias[1] = gy;
	gyro_bias[2] = gz;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(-ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	//generate All Matrix init value 
	init_ekf_matrix(Be,P,X,Q,R);
	
	//Set X init value
	INS_SetState(0,0,0,0,0,0/*alt speed*/,q0,q1,q2,q3,0/*gyro bias x*/,0/*gyro bias y*/,0/*gyro bias z*/,X);
	
	ekf_is_init = true;
}
void ekf_estimator::init(float roll,float pitch,float yaw)
{
	//Init quaternion
	float q0,q1,q2,q3;
	init_quaternion_by_euler(roll,pitch,yaw,&q0,&q1,&q2,&q3);
	init_ekf_matrix(Be,P,X,Q,R);
	INS_SetState(0,0,0,0,0,0/*alt speed*/,q0,q1,q2,q3,0/*gyro bias x*/,0/*gyro bias y*/,0/*gyro bias z*/,X);
	gyro_bias[0]=0;
	gyro_bias[1]=0;
	gyro_bias[2]=0;
	ekf_is_init = true;
}
int ekf_estimator::update(EKF_U u,EKF_Mesurement mesurement,float dT)
{	
	//Declear F G U
	float U[6],F[169],G[117];

	static int wait_convergence;
	//Declear Mesurement
	float Mag_data[3],Pos[3],Vel[2];
	//Declear EulerAngle

	
	if(!ekf_is_init)//if not init , return 
		return -1;
	
	/*caution: the accel sign is all opposite with APM and CC3D*/
	U[0]=u.gyro_x - gyro_bias[0];
	U[1]=u.gyro_y - gyro_bias[1];
	U[2]=u.gyro_z - gyro_bias[2];
	U[3]=-u.accel_x;
	U[4]=-u.accel_y;
	U[5]=-u.accel_z;

	/*caution: the mag_x,mag_y sign is  opposite with APM and CC3D & the mag_z sign is same*/
	Mag_data[0]=-mesurement.Mag_x;
	Mag_data[1]=-mesurement.Mag_y;
	Mag_data[2]= mesurement.Mag_z;
	Pos[0]=mesurement.Pos_GPS_x;
	Pos[1]=mesurement.Pos_GPS_y;
	Pos[2]=mesurement.Pos_Baro_z;//use baro replace gps_d
	Vel[0]=mesurement.Vel_GPS_x;
	Vel[1]=mesurement.Vel_GPS_y;
	
	if(ekf_is_init)
	{
		//To-Do:wait for ekf convergence
		//caculate Jacobian to linear F G
		LinearFG(X,U,F,G);
		
		//Predeict X
		RungeKutta(X,U,dT);
		
		//Predict P
		INS_CovariancePrediction(F,G,Q,dT,P);
		
		//Update P,X
		INS_Correction(Mag_data,Pos,Vel,X,R,P,Be);
		
		/*Fill ekf result with data*/
		ekf_result.Pos_x=X[0];
		ekf_result.Pos_y=X[1];
		ekf_result.Pos_z=X[2];
		ekf_result.Vel_x=X[3];
		ekf_result.Vel_y=X[4];
		ekf_result.Vel_z=X[5];	
		ekf_result.q0=X[6];
		ekf_result.q1=X[7];
		ekf_result.q2=X[8];
		ekf_result.q3=X[9];
		//Transfer quaternion to euler angle
		quaternion_to_euler(1/*is radian:1 OR is Angle:0*/,X[6],X[7],X[8],X[9],&ekf_result.roll,&ekf_result.pitch,&ekf_result.yaw);
		
		
		
	}
	return 1;
}


