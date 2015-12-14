#include "ekf_estimator.h"
#include <string.h>
#include <Protocol/common.h>
#include <HAL/Interface/Interfaces.h>
#include <Algorithm/ekf_lib/inc/init_ekf_matrix.h>
#include <Algorithm/ekf_lib/inc/INS_SetState.h>
#include <Algorithm/ekf_lib/inc/LinearFG.h>
#include <Algorithm/ekf_lib/inc/RungeKutta.h>
#include <Algorithm/ekf_lib/inc/INS_CovariancePrediction.h>
#include <Algorithm/ekf_lib/inc/INS_Correction.h>
#include <Algorithm/ekf_lib/inc/quaternion_to_euler.h>
#include <Algorithm/ekf_lib/inc/init_quaternion_by_euler.h>
#include <Algorithm/ekf_lib/inc/ned2body.h>
#include <Algorithm/ekf_lib/inc/body2ned.h>
ekf_estimator::ekf_estimator()
{
	ekf_is_init=false;
	ekf_is_convergence=false;
	ekf_loopcount=0;
	for(int i=0;i<3;i++)
	{
		gyro_bias[i]=0;
		Be[i]=0;
	}
	for(int i=0;i<13;i++)
		X[i]=0;
	for(int i=0;i<64;i++)
		R[i]=0;
	for(int i=0;i<81;i++)
		Q[i]=0;
	for(int i=0;i<169;i++)
		P[i]=0;
	for(int i=0;i<3;i++)
		for(int j=0;j<50;j++)
			que_window[i][j]=0;
}
ekf_estimator::~ekf_estimator()
{
}
void ekf_estimator::init(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz)
{
	float initialRoll=0, initialPitch=0;
    float cosRoll=0, sinRoll=0, cosPitch=0, sinPitch=0;
    float magX=0, magY=0;
    float initialHdg=0,cosHeading=0, sinHeading=0;
	float q0=0,q1=0,q2=0,q3=0;//quaternion
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
	INS_SetState(0,0,0,0,0,0,q0,q1,q2,q3,0,0,0,X);
	
	ekf_is_init = true;
}
//void ekf_estimator::init(float roll,float pitch,float yaw)
//{
//	//Init quaternion
//	float q0,q1,q2,q3;
//	init_quaternion_by_euler(roll,pitch,yaw,&q0,&q1,&q2,&q3);
//	init_ekf_matrix(Be,P,X,Q,R);
//	INS_SetState(0,0,0,0,0,0/*alt speed*/,q0,q1,q2,q3,0/*gyro bias x*/,0/*gyro bias y*/,0/*gyro bias z*/,X);
//	gyro_bias[0]=0;
//	gyro_bias[1]=0;
//	gyro_bias[2]=0;
//	ekf_is_init = true;
//}
bool ekf_estimator::ekf_is_ready()
{
	if(ekf_is_convergence)
		return true;
	else 
	{
		float mean[3]={0,0,0};
		if(ekf_loopcount<1000)
		ekf_loopcount=ekf_loopcount+1;
		sqrt_variance[0]=0;
		sqrt_variance[1]=0;
		sqrt_variance[2]=0;
		for(int i=0;i<49;i++)
		{
			que_window[0][i]=que_window[0][i+1];
			que_window[1][i]=que_window[1][i+1];
			que_window[2][i]=que_window[2][i+1];
		}
		que_window[0][49]=ekf_result.roll;
		que_window[1][49]=ekf_result.pitch;
		que_window[2][49]=ekf_result.yaw;
		for(int i=0;i<50;i++)
		{
			mean[0]+=que_window[0][i];
			mean[1]+=que_window[1][i];
			mean[2]+=que_window[2][i];
		}
		mean[0]=mean[0]/50;
		mean[1]=mean[1]/50;
		mean[2]=mean[2]/50;
		for(int i=0;i<50;i++)
		{
			sqrt_variance[0]+=(que_window[0][i]-mean[0])*(que_window[0][i]-mean[0]);
			sqrt_variance[1]+=(que_window[1][i]-mean[1])*(que_window[1][i]-mean[1]);
			sqrt_variance[2]+=(que_window[2][i]-mean[2])*(que_window[2][i]-mean[2]);
		}
		sqrt_variance[0]=sqrt(sqrt_variance[0]);
		sqrt_variance[1]=sqrt(sqrt_variance[1]);
		sqrt_variance[2]=sqrt(sqrt_variance[2]);
		
		//if loopcount>800*0.005=4s && sqrt_variance(delt angle)<0.05бу
		if(ekf_loopcount>=800 && sqrt_variance[0]!=0 && sqrt_variance[1]!=0 && sqrt_variance[2]!=0 && sqrt_variance[0]<=0.7f && sqrt_variance[1]<=0.7f && sqrt_variance[2]<=0.7f)
		{
			ekf_is_convergence=1;
			return true;
		}
		return false;
	}

}
int ekf_estimator::update(EKF_U u,EKF_Mesurement mesurement,const float dT)
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
	//check ekf if is ready to arm
	ekf_is_ready();
	return 1;
}
void ekf_estimator::tf_ned2body(const float vector_ned[3], float vector_body[3])
{
	float q[4]={ekf_result.q0,ekf_result.q1,ekf_result.q2,ekf_result.q3};
	ned2body(q,vector_ned,vector_body);
}
void ekf_estimator::tf_body2ned(const float vector_body[3], float vector_ned[3])
{
	float q[4]={ekf_result.q0,ekf_result.q1,ekf_result.q2,ekf_result.q3};
	body2ned(q,vector_body,vector_ned);
}




