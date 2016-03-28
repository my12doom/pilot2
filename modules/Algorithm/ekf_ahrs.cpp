#include "ekf_ahrs.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

ekf_ahrs::ekf_ahrs()
{
	x = matrix(7, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	P = matrix::diag(7, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 );

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
	matrix o(3,1,
		-2.f * (x[1]*x[3] - x[0]*x[2]),
		-2.f * (x[2]*x[3] + x[0]*x[1]),
		-(x[0]*x[0] - x[1]*x[1] - x[2]*x[2] + x[3]*x[3])
		);

	return o;
}

int ekf_ahrs::update(float a[3], float g[3], float dt)
{
	matrix Q = matrix::diag(7, 1e-5, 1e-5, 1e-5, 1e-5, 1e-7, 1e-7, 1e-7);

	matrix R = matrix::diag(3, 0.1, 0.1, 0.1);

	float dt2 = dt/2;
	matrix F(7,7,
		1.0, dt2*-g[0], dt2*-g[1], dt2*-g[2], dt2*-x[1], dt2*-x[2], dt2*-x[3],
		dt2*g[0], 1.0, dt2*g[2], dt2*-g[1], dt2*x[0], dt2*-x[3], dt2*x[2],
		dt2*g[1], dt2*-g[2], 1.0, dt2*g[0], dt2*x[3], dt2*x[0], dt2*-x[1],
		dt2*g[2], dt2*g[1], dt2*-g[0], 1.0, dt2*-x[2], dt2*x[1], dt2*x[0],


		0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	matrix H = matrix(3,7,
		2*x[2], 2*-x[3], 2*x[0], 2*-x[1], 0.0, 0.0, 0.0,
		-2*x[1], 2*-x[0], 2*-x[3], 2*-x[2], 0.0, 0.0, 0.0,
		-2*x[0], 2*x[1], 2*x[2], -2*x[3], 0.0, 0.0, 0.0);

	matrix Hx = H * x;

	float a_len = 1.0/sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	matrix zk(3,1,a[0]*a_len, a[1] * a_len, a[2] * a_len);


	matrix x1 = F * x;
	matrix predicted_zk = h(x1);
	matrix P1 = F * P * F.transpos() + Q;
	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();
	x = x1 + K*(zk - h(x1));
	P = (matrix(P1.m) - K*H) * P1;


	// renorm
	float sqq = 1.0f/sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
	x[0] *= sqq;
	x[1] *= sqq;
	x[2] *= sqq;
	x[3] *= sqq;


	return 0;
}

extern "C" void quaternion_to_euler(signed char is_radian, float q0, float q1, float q2,
						 float q3, float *roll, float *pitch, float *yaw);

int test_imu_simple()
{
#ifdef WIN32
	FILE *f = fopen("Z:\\imu_simple.csv", "wb");
	if (f)
	{
		fprintf(f, "t,bias[0],bias[1],bias[2],gyro[0],gyro[1],gyro[2]\n");
	}
#endif

	for(int i=0; i<15; i++)
	{
		ekf_ahrs imu;

		float dt = 3e-3;

		for(float t=0; t<100; t+=dt)
		{
			float a[3] = {-1,-1,-1};
			float g[3] = {0.002, 0.003, 0.004};

			// noise
			for(int k=0; k<3; k++)
			{
				a[k] += (rand() % 255 - 127)/255.0f * 0.2f;
				g[k] += (rand() % 255 - 127)/255.0f * 0.0002f;
			}

			imu.update(a, g, dt);

			static int m = 0;
			if (f && /*i == 0 &&*/ m++ %25 == 0)
			{
				matrix &x = imu.x;
				float roll, pitch, yaw;
				float roll_raw = atan2(-a[1], -a[2]) * 180 / 3.14159;
				float pitch_raw = atan2(a[0], (-a[2] > 0 ? 1 : -1) * sqrt(a[1]*a[1] + a[2]*a[2])) * 180 / 3.14159;

				quaternion_to_euler(0, x[0], x[1], x[2], x[3], &roll, &pitch, &yaw);
				fprintf(f, "%f,%f,%f,%f,%f,%f,%f,%f\r\n", t+i*100, x[4], x[5], x[6], roll, pitch, roll_raw, pitch_raw);
			}
		}

		float a[3] = {-1,-1,-1};
		float g[3] = {0.002, 0.003, 0.004};
		imu.update(a, g, dt);


		matrix &x = imu.x;
		printf("q:%f,%f,%f,%f\tbias:%f,%f,%f\n", x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
	}

	return 0;
}
