#include <stdint.h>
#include <string.h>
#include "ahrs.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
//! Auxiliary variables to reduce number of repeated operations
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
float Rot_matrix[9];
float euler[3];
float NED2BODY[3][3];
float BODY2NED[3][3];
float acc_ned[3];

static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static bool bFilterInit = false;
static float ground_mag_length;
static float mag_tolerate = 0.25f;

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);
int inverse_matrix3x3(const float src[3][3], float dst[3][3]);

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
	ground_mag_length = sqrt(mx*mx + my*my + mz*mz);


	gyro_bias[0] = -gx;
	gyro_bias[1] = -gy;
	gyro_bias[2] = -gz;

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
	
	bFilterInit = true;
}

void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float twoKpMag, float twoKiMag, float dt) 
{
	float recipNorm;
	float halfexA = 0.0f, halfeyA = 0.0f, halfezA = 0.0f;
	float halfexM = 0.0f, halfeyM = 0.0f, halfezM = 0.0f;
	float acc_bodyframe[3] = {ax, ay, az};
	float g_force = sqrt(ax*ax + ay*ay + az*az) / 9.8065f;
	bool g_force_ok = g_force > 0.85f && g_force < 1.15f;
	float mag_length = sqrt(mx*mx + my*my + mz*mz);
	if (ground_mag_length != 0)
	{
		// check for magnetic interference
		float mag_diff = fabs(mag_length / ground_mag_length - 1.0f);
		if ( mag_diff < mag_tolerate)
		{
			mag_tolerate = mag_diff;
		}
		else
		{
			mx = my = mz = 0;
			mag_tolerate += 0.05f * dt;
			//TRACE("warning: possible magnetic interference");
		}

	}
	else
	{
		// no initial mag data
		mag_length = 0;
	}

	// Make filter converge to initial solution faster
	// This function assumes you are in static position.
	// WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
	if(bFilterInit == false) {
		NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz, gx, gy, gz);
		bFilterInit = true;
	}
        	
	//! If magnetometer measurement is available, use it.
	if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
		float hx, hy, hz, bx, bz;
		float halfwx, halfwy, halfwz;
	
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
    	halfexM = (my * halfwz - mz * halfwy);
    	halfeyM = (mz * halfwx - mx * halfwz);
    	halfezM = (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(g_force_ok && !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float halfvx, halfvy, halfvz;
	
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfexA += ay * halfvz - az * halfvy;
		halfeyA += az * halfvx - ax * halfvz;
		halfezA += ax * halfvy - ay * halfvx;
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

	Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
	Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
	Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
	Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
	Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
	Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
	Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
	Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
	Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

	memcpy(&NED2BODY, Rot_matrix, sizeof(float)*9);
	inverse_matrix3x3(NED2BODY, BODY2NED);

	/* transform acceleration vector from body frame to NED frame */
	for (int i = 0; i < 3; i++) {
		acc_ned[i] = 0.0f;

		for (int j = 0; j < 3; j++) {
			acc_ned[i] += BODY2NED[i][j] * acc_bodyframe[j];
		}
	}
	acc_ned[2] -= 9.8065f;


//   	LOGE("accz=%f/%f, acc=%f,%f,%f, raw=%f,%f,%f\n", accz_NED, accelz, acc[0], acc[1], acc[2], BODY2NED[0][0], BODY2NED[0][1], BODY2NED[0][2]);

	//1-2-3 Representation.
	//Equation (290) 
	//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
	// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
	euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll
	euler[1] = -asinf(Rot_matrix[2]);	//! Pitch
	euler[2] = atan2f(-Rot_matrix[1], -Rot_matrix[0]);		//! Yaw, 0 = north, PI/-PI = south, PI/2 = east, -PI/2 = west
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

// see http://mathworld.wolfram.com/MatrixInverse.html
int inverse_matrix3x3(const float src[3][3], float dst[3][3])
{
	float det = +src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) 
		-src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0])
		+src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	det = 1.0f/det;

	dst[0][0] =  det * (src[1][1]*src[2][2] - src[1][2] * src[2][1]);
	dst[0][1] = det * (src[0][2]*src[2][1] - src[0][1] * src[2][2]);
	dst[0][2] =  det * (src[0][1]*src[1][2] - src[0][2] * src[1][1]);

	dst[1][0] = det * (src[1][2]*src[2][0] - src[1][0] * src[2][2]);
	dst[1][1] =  det * (src[0][0]*src[2][2] - src[0][2] * src[2][0]);
	dst[1][2] = det * (src[0][2]*src[1][0] - src[0][0] * src[1][2]);

	dst[2][0] =  det * (src[1][0]*src[2][1] - src[1][1] * src[2][0]);
	dst[2][1] = det * (src[0][1]*src[2][0] - src[0][0] * src[2][1]);
	dst[2][2] =  det * (src[0][0]*src[1][1] - src[0][1] * src[1][0]);

	return 0;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
