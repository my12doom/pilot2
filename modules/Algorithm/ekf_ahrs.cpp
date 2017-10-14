#include "ekf_ahrs.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <Protocol/common.h>
#include <math/quaternion.h>

ekf_ahrs::ekf_ahrs()
{
	x = matrix(7, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	P = matrix::diag(7, 100.0, 100.0, 100.0, 100.0, 1e+1, 1e+1, 1e+1 );

	motion_acc.set_threshold(0.2);
	motion_gyro.set_threshold(5*PI/180);

	still = false;
	inited = false;
}

ekf_ahrs::~ekf_ahrs()
{

}

/*
dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

q0 += dt*dq0;
q1 += dt*dq1;
q2 += dt*dq2;
q3 += dt*dq3;

NED2BODY[0][0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
NED2BODY[0][1] = 2.f * (q1*q2 + q0*q3);	// 12
NED2BODY[0][2] = 2.f * (q1*q3 - q0*q2);	// 13
NED2BODY[1][0] = 2.f * (q1*q2 - q0*q3);	// 21
NED2BODY[1][1] = q0q0 - q1q1 + q2q2 - q3q3;// 22
NED2BODY[1][2] = 2.f * (q2*q3 + q0*q1);	// 23
NED2BODY[2][0] = 2.f * (q1*q3 + q0*q2);	// 31
NED2BODY[2][1] = 2.f * (q2*q3 - q0*q1);	// 32
NED2BODY[2][2] = q0q0 - q1q1 - q2q2 + q3q3;// 33

*/

matrix ekf_ahrs::h(matrix &x)
{
	matrix o(still ? 9: 6,1,
		-2.f * (x[1]*x[3] - x[0]*x[2]),						// 13
		-2.f * (x[2]*x[3] + x[0]*x[1]),						// 23
		-(x[0]*x[0] - x[1]*x[1] - x[2]*x[2] + x[3]*x[3]),	// 33
		x[0]*x[0] + x[1]*x[1] - x[2]*x[2] - x[3]*x[3],		// 11
		2.f * (x[1]*x[2] - x[0]*x[3]),						// 21
		2.f * (x[1]*x[3] + x[0]*x[2]),						// 31
		x[4],
		x[5],
		x[6]
		);

	return o;
}

matrix ekf_ahrs::f(matrix &x, float gx, float gy, float gz, float dt)
{
	float q0 = x[0];
	float q1 = x[1];
	float q2 = x[2];
	float q3 = x[3];

	gx += x[4];
	gy += x[5];
	gz += x[6];

	float dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
	float dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
	float dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
	float dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx); 

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;

	return matrix(7,1,q0, q1, q2, q3, x[4], x[5], x[6]);
}

int ekf_ahrs::update(float a[3], float g[3], float mag[3], float dt, bool use_mag)
{
	if (!inited)
		init(a, g, mag);

	// motion detection
	vector va = {a[0], a[1], a[2]};
	vector vg = {g[0], g[1], g[2]};
	motion_acc.new_data(va);
	motion_gyro.new_data(vg);

	still = motion_acc.get_average(NULL) > 100 && motion_gyro.get_average(NULL) > 100;
// 	still = false;

	matrix Q = matrix::diag(7, 1e-8, 1e-8, 1e-8, 1e-8, 1e-12, 1e-12, 1e-12);

	matrix R = still ? matrix::diag(9, 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4) 
		: (use_mag ? matrix::diag(6, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1) : matrix::diag(6, 1e-1, 1e-1, 1e-1, 1e+5, 1e+5, 1e+5));

	float dt2 = dt/2;
	matrix F(7,7,
		1.0, dt2*-g[0], dt2*-g[1], dt2*-g[2], dt2*-x[1], dt2*-x[2], dt2*-x[3],
		dt2*g[0], 1.0, dt2*g[2], dt2*-g[1], dt2*x[0], dt2*-x[3], dt2*x[2],
		dt2*g[1], dt2*-g[2], 1.0, dt2*g[0], dt2*x[3], dt2*x[0], dt2*-x[1],
		dt2*g[2], dt2*g[1], dt2*-g[0], 1.0, dt2*-x[2], dt2*x[1], dt2*x[0],


		0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	matrix H = matrix(still?9:6,7,
		2*x[2], 2*-x[3], 2*x[0], 2*-x[1], 0.0, 0.0, 0.0,
		-2*x[1], 2*-x[0], 2*-x[3], 2*-x[2], 0.0, 0.0, 0.0,
		-2*x[0], 2*x[1], 2*x[2], -2*x[3], 0.0, 0.0, 0.0,
		2*x[0], 2*x[1], -2*x[2], -2*x[3], 0.0, 0.0, 0.0,
		-2*x[3], 2*x[2], 2*x[1], -2*x[0], 0.0, 0.0, 0.0,
		2*x[2], 2*x[3], 2*x[0], 2*x[1], 0.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 1.0, 0.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0, 1.0, 0.0,
		0.0,0.0,0.0,0.0, 0.0, 0.0, 1.0
		);

	matrix Hx = H * x;

	float a_len = 1.0/sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);

	float mag_0z[3] = {mag[0], mag[1], mag[2]};
	float q[4] = {x[0], x[1], x[2], x[3]};
	remove_mag_ned_z(mag_0z, q);
	float m_len = 1.0/sqrt(mag_0z[0] * mag_0z[0] + mag_0z[1] * mag_0z[1] + mag_0z[2] * mag_0z[2]);
	zk = matrix(still?9:6,1,a[0]*a_len, a[1] * a_len, a[2] * a_len, mag_0z[0] * m_len, mag_0z[1] * m_len, mag_0z[2] * m_len, -g[0], -g[1], -g[2]);


	matrix x2 = F * x;
	matrix x1 = x2;
	matrix predicted_zk = h(x1);
	matrix P1 = F * P * F.transpos() + Q;
	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();

	predicted = h(x1);
	matrix residual = zk - predicted;

	matrix p0(4,1,0.0,0.0,0.0,1.0);
	matrix p1(4,1,0.0,0.0,0.0,-1.0);

	matrix pp0 = h(p0);
	matrix pp1 = h(p1);

	x = x1 + K*(zk - h(x1));
	P = (matrix(P1.m) - K*H) * P1;

	// too high gyro bias detection
	// modern gyros won't have more than 5 degree/s total bias
// 	if (x[4] * x[4] + x[5] * x[5] + x[6] * x[6] > (15*15*PI*PI/180/180))
// 	{
// 		P[4*8] += dt;
// 		P[5*8] += dt;
// 		P[6*8] += dt;
// 		P[7*8] += dt;
// 	}


	// renorm
	float sqq = 1.0f/sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
	x[0] *= sqq;
	x[1] *= sqq;
	x[2] *= sqq;
	x[3] *= sqq;

	if (x[0] < 0)
	{
		x[0] = -x[0];
		x[1] = -x[1];
		x[2] = -x[2];
		x[3] = -x[3];
	}


	return 0;
}

int ekf_ahrs::init(float a[3], float g[3], float m[3])
{
	inited = true;

	float roll = atan2(-a[1], -a[2]) * 180 / 3.14159;
	float pitch = atan2(a[0], (-a[2] > 0 ? 1 : -1) * sqrt(a[1]*a[1] + a[2]*a[2])) * 180 / 3.14159;

	float cosRoll = cosf(roll);
	float sinRoll = sinf(roll);
	float cosPitch = cosf(pitch);
	float sinPitch = sinf(pitch);

	float magX = m[0] * cosPitch + m[1] * sinRoll * sinPitch + m[2] * cosRoll * sinPitch;
	float magY = m[1] * cosRoll - m[2] * sinRoll;
	float initialHdg = atan2f(-magY, magX);
	float cosHeading = cosf(initialHdg * 0.5f);
	float sinHeading = sinf(initialHdg * 0.5f);

	x[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	x[1] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	x[2] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	x[3] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	x[4] = g[0];
	x[5] = g[1];
	x[6] = g[2];

	return 0;
}


void ekf_ahrs::remove_mag_ned_z(float *mag_body, float *q)
{
	float BODY2NED[3][3];
	float NED2BODY[3][3];

	float q0 = q[0];
	float q1 = q[1];
	float q2 = q[2];
	float q3 = q[3];

	NED2BODY[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;// 11
	NED2BODY[0][1] = 2.f * (q1*q2 + q0*q3);	// 12
	NED2BODY[0][2] = 2.f * (q1*q3 - q0*q2);	// 13
	NED2BODY[1][0] = 2.f * (q1*q2 - q0*q3);	// 21
	NED2BODY[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;// 22
	NED2BODY[1][2] = 2.f * (q2*q3 + q0*q1);	// 23
	NED2BODY[2][0] = 2.f * (q1*q3 + q0*q2);	// 31
	NED2BODY[2][1] = 2.f * (q2*q3 - q0*q1);	// 32
	NED2BODY[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;// 33

	BODY2NED[0][0] = NED2BODY[0][0];
	BODY2NED[0][1] = NED2BODY[1][0];
	BODY2NED[0][2] = NED2BODY[2][0];

	BODY2NED[1][0] = NED2BODY[0][1];
	BODY2NED[1][1] = NED2BODY[1][1];
	BODY2NED[1][2] = NED2BODY[2][1];

	BODY2NED[2][0] = NED2BODY[0][2];
	BODY2NED[2][1] = NED2BODY[1][2];
	BODY2NED[2][2] = NED2BODY[2][2];

	// transform vector from body frame to NED frame, discard z component
	float x = BODY2NED[0][0] * mag_body[0] + BODY2NED[0][1] * mag_body[1] + BODY2NED[0][2] * mag_body[2];
	float y = BODY2NED[1][0] * mag_body[0] + BODY2NED[1][1] * mag_body[1] + BODY2NED[1][2] * mag_body[2];

	// transform back to body frame
	mag_body[0] = NED2BODY[0][0] * x + NED2BODY[0][1] * y; // + NED2BODY[0][2] * 0;
	mag_body[1] = NED2BODY[1][0] * x + NED2BODY[1][1] * y; // + NED2BODY[1][2] * 0;
	mag_body[2] = NED2BODY[2][0] * x + NED2BODY[2][1] * y; // + NED2BODY[2][2] * 0;
}

int ekf_ahrs::get_euler(float *euler)
{

	float q[4] = {x[0], x[1], x[2], x[3]};
	Quaternion2RPY(q, euler);

	return 0;
}
/*
int test_ekf_ahrs()
{
#ifdef WIN32
	FILE *f = fopen("Z:\\imu_simple.csv", "wb");
	if (f)
	{
		fprintf(f, "t,bias[0],bias[1],bias[2],euler[0],euler[1],euler[2],raw[0],raw[1],raw[2]\n");
	}
#endif

	for(int i=0; i<15; i++)
	{
		ekf_ahrs imu;

		float dt = 3e-3;

		for(float t=0; t<100; t+=dt)
		{
			float a[3] = {0,0,-1};
			float g[3] = {0.002, 0.003, 0.004};
			float m[3] = {1,0,0};

			// noise
			for(int k=0; k<3; k++)
			{
				a[k] += (rand() % 255 - 127)/255.0f * 0;
				g[k] += (rand() % 255 - 127)/255.0f * 0;
				m[k] += (rand() % 255 - 127)/255.0f * 0;

				if (t > 49.9 && t < 50)
				{
					imu.x[0] = 1.0;
					imu.x[1] = 0;
					imu.x[2] = 0;
					imu.x[3] = 0;
				}

				if (t > 50 && t< 52)
					g[1] += 120*PI/180;
				else if (t > 52 && t < 54)
					g[1] += -120*PI/180;
			}

			imu.update(a, g, m, dt, true);

			static int n = 0;
			if (f && n++ %25 == 0)
			{
				matrix &x = imu.x;
				float roll, pitch, yaw;
				Quaternion2RPY(&x[0], &roll);

				float roll_raw = atan2(-a[1], -a[2]) * 180 / 3.14159;
				float pitch_raw = atan2(a[0], (-a[2] > 0 ? 1 : -1) * sqrt(a[1]*a[1] + a[2]*a[2])) * 180 / 3.14159;

				float cosRoll = cosf(roll);
				float sinRoll = sinf(roll);
				float cosPitch = cosf(pitch);
				float sinPitch = sinf(pitch);

				float magX = m[0] * cosPitch + m[1] * sinRoll * sinPitch + m[2] * cosRoll * sinPitch;
				float magY = m[1] * cosRoll - m[2] * sinRoll;
				float initialHdg = atan2f(-magY, magX);


				fprintf(f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", t+i*100, x[4]*180/PI, x[5]*180/PI, x[6]*180/PI, roll, pitch, yaw, roll_raw, pitch_raw, initialHdg);
			}
		}

		float a[3] = {-1,-1,-1};
		float g[3] = {0.002, 0.003, 0.004};
		float m[3] = {-1,0,0};
		imu.update(a, g, m, dt, true);


		matrix &x = imu.x;
		printf("q:%f,%f,%f,%f\tbias:%f,%f,%f\n", x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
	}

	return 0;
}
*/