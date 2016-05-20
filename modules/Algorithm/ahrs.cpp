#include "ahrs.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <Protocol/common.h>

//---------------------------------------------------------------------------------------------------
//! Auxiliary variables to reduce number of repeated operations
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
float euler[3];
float NED2BODY[3][3];
float BODY2NED[3][3];
float acc_ned[3];
float halfvx, halfvy, halfvz;
float acc_horizontal[3];
float raw_yaw;
bool mag_ok = true;
float err_a[3];
float err_m[3];

static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static bool bFilterInit = false;
static bool has_mag;
static float mag_tolerate = 0.25f;
#define min_mag_tolerate 0.25f

//---------------------------------------------------------------------------------------------------
// Function declarations
float invSqrt(float x);
void remove_down_component(float &bx, float &by, float &bz);			// remove earth frame down component of a body frame vector
int inverse_matrix3x3(const float src[3][3], float dst[3][3]);
void transpos_matrix3x3(const float src[3][3], float dst[3][3]);
static inline float fmax(float a, float b)
{
	return a>b?a:b;
}

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm initialization
//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;
	has_mag = mx!=0 || my !=0 || mz !=0;


	gyro_bias[0] = -gx;
	gyro_bias[1] = -gy;
	gyro_bias[2] = -gz;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, (-az > 0 ? 1 : -1) * sqrt(ay*ay + az*az));

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

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
	
	halfvx = q1q3 - q0q2;
	halfvy = q0q1 + q2q3;
	halfvz = q0q0 - 0.5f + q3q3;

	
	bFilterInit = true;
}

void NonlinearSO3AHRSupdate(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz, float twoKp, float twoKi, float twoKpMag, float twoKiMag, float dt, float gps_ax, float gps_ay, float gps_az)
{
	float recipNorm;
	float halfexA = 0.0f, halfeyA = 0.0f, halfezA = 0.0f;
	float halfexM = 0.0f, halfeyM = 0.0f, halfezM = 0.0f;
	float acc_bodyframe[3] = {-ax, -ay, -az};

	ax -= gps_ax;
	ay -= gps_ay;
	az -= gps_az;

	float g_force = sqrt(ax*ax + ay*ay + az*az) / G_in_ms2;
	bool g_force_ok = g_force > 0.85f && g_force < 1.15f;

	if (has_mag)
	{

		// check for magnetic interference		
		// by mag field length and residual
		float mag_length = sqrt(mx*mx + my*my + mz*mz);
		float mag_times = mag_length > 500.0f ? (mag_length / 500.0f) : (500.0f/mag_length);
		float mag_residual = sqrt(err_m[0] * err_m[0] + err_m[1] * err_m[1] + err_m[2] * err_m[2]);

		bool mag_length_ok = mag_ok ? (mag_times < 3.0f) : (mag_times < 1.5f);
		bool mag_residual_ok = mag_ok ? (mag_residual < 0.2f) : (mag_residual < 0.05f);

		if (mag_ok && !(mag_residual_ok && mag_length_ok))
		{
			int code =  (mag_length_ok ? 0: 1) + (mag_residual_ok ? 0 : 2);
			static const char *codes[] = 
			{
				"none", "mag field", "mag residual", "both",
			};
			LOGE("AHRS_CF: mag interference %s\n", codes[code]);
			mag_ok = false;
		}
		else if (!mag_ok && mag_residual_ok && mag_length_ok)
		{
			LOGE("AHRS_CF: mag restored\n");
			mag_ok = true;
		}
	}

	// Make filter converge to initial solution faster
	// This function assumes you are in static position.
	// WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
	if(bFilterInit == false) {
		NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz, gx, gy, gz);
		bFilterInit = true;
	}
        	
	//! If magnetometer measurement is available, use it.
	if(has_mag && !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
		float hx, hy, hz, bx, bz;
		float halfwx, halfwy, halfwz;
	
    	// discard Z component in NED frame to avoid large attitude change due to magnet interference, especially during initialization.
		remove_down_component(mx, my, mz);

		// Normalise magnetometer measurement
		// Will sqrt work better? PX4 system is powerful enough?
    	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    	mx *= recipNorm;
    	my *= recipNorm;
    	mz *= recipNorm;
    
    	// Reference direction of Earth's magnetic field
    	hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    	hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    	bx = sqrt(hx * hx + hy * hy);
    	bz = hz;
    
    	// Estimated direction of magnetic field
    	halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    	halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    	halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    	// Error is sum of cross product between estimated direction and measured direction of field vectors
		// apply only if mag_ok
		err_m[0] = (my * halfwz - mz * halfwy);
		err_m[1] = (mz * halfwx - mx * halfwz);
		err_m[2] = (mx * halfwy - my * halfwx);

		if (mag_ok)
		{
    		halfexM = err_m[0];
    		halfeyM = err_m[1];
    		halfezM = err_m[2];
		}
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	g_force_ok = true;
	if(g_force_ok && !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
	
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and magnetic field
		halfvx = -(q1q3 - q0q2);
		halfvy = -(q0q1 + q2q3);
		halfvz = -(q0q0 - 0.5f + q3q3);
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfexA += ay * halfvz - az * halfvy;
		halfeyA += az * halfvx - ax * halfvz;
		halfezA += ax * halfvy - ay * halfvx;

		err_a[0] = halfexA;
		err_a[1] = halfeyA;
		err_a[2] = halfezA;
	}

	// Compute and apply integral feedback if enabled
	gyro_bias[0] += (twoKi * halfexA /*+ twoKiMag * halfexM*/) * dt;	// integral error scaled by Ki
	gyro_bias[1] += (twoKi * halfeyA /*+ twoKiMag * halfeyM*/) * dt;
	gyro_bias[2] += (twoKi * halfezA + twoKiMag * halfezM) * dt;
	
	// apply integral feedback
	gx += gyro_bias[0];
	gy += gyro_bias[1];
	gz += gyro_bias[2];

	// Apply proportional feedback
	gx += twoKp * halfexA + twoKpMag * halfexM;
	gy += twoKp * halfeyA + twoKpMag * halfeyM;
	gz += twoKp * halfezA + twoKpMag * halfezM;
	
	//! Integrate rate of change of quaternion
#if 0
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
#endif 

	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
   	q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

	NED2BODY[0][0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
	NED2BODY[0][1] = 2.f * (q1*q2 + q0*q3);	// 12
	NED2BODY[0][2] = 2.f * (q1*q3 - q0*q2);	// 13
	NED2BODY[1][0] = 2.f * (q1*q2 - q0*q3);	// 21
	NED2BODY[1][1] = q0q0 - q1q1 + q2q2 - q3q3;// 22
	NED2BODY[1][2] = 2.f * (q2*q3 + q0*q1);	// 23
	NED2BODY[2][0] = 2.f * (q1*q3 + q0*q2);	// 31
	NED2BODY[2][1] = 2.f * (q2*q3 - q0*q1);	// 32
	NED2BODY[2][2] = q0q0 - q1q1 - q2q2 + q3q3;// 33

	//memcpy(&NED2BODY, Rot_matrix, sizeof(float)*9);
	transpos_matrix3x3(NED2BODY, BODY2NED);

	/* transform acceleration vector from body frame to NED frame */
	for (int i = 0; i < 3; i++) {
		acc_ned[i] = 0.0f;

		for (int j = 0; j < 3; j++) {
			acc_ned[i] += BODY2NED[i][j] * acc_bodyframe[j];
		}
	}
	acc_ned[2] -= G_in_ms2;

	//1-2-3 Representation.
	//Equation (290) 
	//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
	// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
	euler[0] = atan2f(NED2BODY[1][2], NED2BODY[2][2]);	//! Roll
	euler[1] = -asinf(NED2BODY[0][2]);	//! Pitch
	euler[2] = atan2f(NED2BODY[0][1], NED2BODY[0][0]);		//! Yaw, 0 = north, PI/-PI = south, PI/2 = east, -PI/2 = west

	// NED frame to horizontal body frame
	float cos_yaw = cos(euler[2]);
	float sin_yaw = sin(euler[2]);
	acc_horizontal[0] = cos_yaw * acc_ned[0] + sin_yaw * acc_ned[1];
	acc_horizontal[1] = -sin_yaw * acc_ned[0] + cos_yaw * acc_ned[1];
	acc_horizontal[2] = acc_ned[2];

	// tilt compensated raw mag yaw
	float cosRoll = cosf(euler[0]);
    float sinRoll = sinf(euler[0]);
    float cosPitch = cosf(euler[1]+PI);
    float sinPitch = sinf(euler[1]+PI);

    float magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
    float magY = my * cosRoll - mz * sinRoll;

    raw_yaw = atan2f(-magY, magX);


	//   	LOGE("accz=%f/%f, acc=%f,%f,%f, raw=%f,%f,%f\n", accz_NED, accelz, acc[0], acc[1], acc[2], BODY2NED[0][0], BODY2NED[0][1], BODY2NED[0][2]);
	TRACE("\raccel fr:%f,%f,  ned:%f,%f,%f,", acc_horizontal[0], acc_horizontal[1], acc_ned[0], acc_ned[1], acc_ned[2]);
}

void NonlinearSO3AHRSreset_mag(float mx, float my, float mz)
{
	float cosRoll = cosf(euler[0] * 0.5f);
	float sinRoll = sinf(euler[0] * 0.5f);

	float cosPitch = cosf(euler[1] * 0.5f);
	float sinPitch = sinf(euler[1] * 0.5f);

	float magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
	float magY = my * cosRoll - mz * sinRoll;

	float initialHdg = atan2f(-magY, magX);
	float cosHeading = cosf(initialHdg*0.5f);
	float sinHeading = sinf(initialHdg*0.5f);

	q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	// auxillary variables to reduce number of repeated operations, for 1st pass
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	halfvx = q1q3 - q0q2;
	halfvy = q0q1 + q2q3;
	halfvz = q0q0 - 0.5f + q3q3;

	mag_ok = true;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void transpos_matrix3x3(const float src[3][3], float dst[3][3])
{
	dst[0][0] =  src[0][0];
	dst[0][1] = src[1][0];
	dst[0][2] =  src[2][0];

	dst[1][0] = src[0][1];
	dst[1][1] =  src[1][1];
	dst[1][2] = src[2][1];

	dst[2][0] =  src[0][2];
	dst[2][1] = src[1][2];
	dst[2][2] =  src[2][2];
}

void remove_down_component(float &bx, float &by, float &bz)			// remove earth frame down factor of a body frame vector
{
	// transform vector from body frame to NED frame, discard z component
	float x = BODY2NED[0][0] * bx + BODY2NED[0][1] * by + BODY2NED[0][2] * bz;
	float y = BODY2NED[1][0] * bx + BODY2NED[1][1] * by + BODY2NED[1][2] * bz;

	// transform back to body frame
	bx = NED2BODY[0][0] * x + NED2BODY[0][1] * y; // + NED2BODY[0][2] * 0;
	by = NED2BODY[1][0] * x + NED2BODY[1][1] * y; // + NED2BODY[1][2] * 0;
	bz = NED2BODY[2][0] * x + NED2BODY[2][1] * y; // + NED2BODY[2][2] * 0;
}


//====================================================================================================
// END OF CODE
//====================================================================================================
