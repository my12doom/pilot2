#pragma once

#include <math.h>

#ifndef M_PI
static float M_PI = acos(-1.0f);
#endif

// ****** find quaternion from roll, pitch, yaw ********
static void RPY2Quaternion(const float rpy[3], float q[4])
{
    float phi, theta, psi;
    float cphi, sphi, ctheta, stheta, cpsi, spsi;

    phi    = (rpy[0] / 2);
    theta  = (rpy[1] / 2);
    psi    = (rpy[2] / 2);
    cphi   = cosf(phi);
    sphi   = sinf(phi);
    ctheta = cosf(theta);
    stheta = sinf(theta);
    cpsi   = cosf(psi);
    spsi   = sinf(psi);

    q[0]   = cphi * ctheta * cpsi + sphi * stheta * spsi;
    q[1]   = sphi * ctheta * cpsi - cphi * stheta * spsi;
    q[2]   = cphi * stheta * cpsi + sphi * ctheta * spsi;
    q[3]   = cphi * ctheta * spsi - sphi * stheta * cpsi;

    if (q[0] < 0) { // q0 always positive for uniqueness
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
}

/**
 * @brief Compute the inverse of a quaternion
 * @param [in][out] q The matrix to invert
 */
static void quat_inverse(float q[4])
{
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

/**
 * @brief Multiply two quaternions into a third
 * @param[in] q1 First quaternion
 * @param[in] q2 Second quaternion
 * @param[out] qout Output quaternion
 */
static void quat_mult(const float q1[4], const float q2[4], float qout[4])
{
    qout[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    qout[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    qout[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    qout[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

// ****** find roll, pitch, yaw from quaternion ********
static void Quaternion2RPY(const float q[4], float rpy[3])
{
    float R13, R11, R12, R23, R33;
    float q0s = q[0] * q[0];
    float q1s = q[1] * q[1];
    float q2s = q[2] * q[2];
    float q3s = q[3] * q[3];

    R13    = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    R11    = q0s + q1s - q2s - q3s;
    R12    = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    R23    = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    R33    = q0s - q1s - q2s + q3s;

    rpy[1] = (asinf(-R13)); // pitch always between -pi/2 to pi/2
    rpy[2] = (atan2f(R12, R11));
    rpy[0] = (atan2f(R23, R33));

    // TODO: consider the cases where |R13| ~= 1, |pitch| ~= pi/2
}

static float radian_add(float a, float b)
{
	a += b;
	if (a>=M_PI)
		a -= 2*M_PI;
	if (a<-M_PI)
		a += 2*M_PI;
	
	return a;
}

static void Quaternion2BFAngle(const float q[4], float angle[3])
{
	angle[0] = q[1];
	angle[1] = q[2];
	angle[2] = q[3];

	float len = sqrt(angle[0]*angle[0] + angle[1]*angle[1] + angle[2]*angle[2]);
	if (len >= 1.0e-12f) {
		float invl = 1.0f/len;
		float qp = radian_add(2.0f * atan2f(len,q[0]), 0);
		angle[0] *= qp * invl;
		angle[1] *= qp * invl;
		angle[2] *= qp * invl;
	}
}