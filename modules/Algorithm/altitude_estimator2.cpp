#include "altitude_estimator2.h"
#include <string.h>
#include <Protocol/common.h>
#include <HAL/Interface/Interfaces.h>
#include <utils/log.h>

#define sonar_step_threshold 0.15f

static matrix Q = matrix::diag(5, 4e-6, 1e-6, 1e-3, 1e-7, 1e-7);


altitude_estimator2::altitude_estimator2()
:x(5, 1, 0.0, 0.0, 0.0, 0.0, 0.9)				// [alt, climb, abias, surface_alt, sonar_scale]
,static_mode(true)
,compensate_land_effect(false)
,P(matrix::diag(5, 200.0, 200.0, 200.0, 200.0, 200.0))
{
}

altitude_estimator2::~altitude_estimator2()
{

}

int altitude_estimator2::update(float accelz, float baro, float sonar, float dt)
{
	if (dt > 0.2f || dt < 0)
		return -1;

	// sonar step response handling
	if (!isnan(sonar))
	{
		if (sonar_used && fabs(sonar - last_valid_sonar) >  sonar_step_threshold)
		{
			LOGE("estimator2: step response: %f -->> %f\n", last_valid_sonar, sonar);
			x[3] += (sonar - last_valid_sonar)/x[4];
			P[18] = 200;
		}

		last_valid_sonar = sonar;
	}
	
	// sonar switching
	if (!isnan(sonar) == sonar_used)				// reset ticker if sonar state didn't changed
		sonar_ticker = 0;

	if (sonar_ticker < 0.3f)
	{
		sonar_ticker += dt;

		if (sonar_ticker > 0.3f)
		{
			// sonar state changed more than 0.3 second.
			if (sonar_used)
			{
				sonar_used = false;
				LOGE("estimator2: changed to baro\n");
			}
			else
			{
				sonar_used = true;
				x[3] = x[0] - last_valid_sonar/x[4];
				P[18] = 200;
				LOGE("estimator2: changed to sonar\n");
			}
		}
	}

	float dtsq2 = 0.5f * dt * dt;
	matrix F
	(
		5,5,
		1.0, dt, dtsq2, 0.0, 0.0,
		0.0, 1.0, dt, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0
	);
	matrix B
		(5,1,
		dtsq2,
		dt,
		0.0,
		0.0,
		0.0
		);
	matrix u(1,1,accelz);

	matrix H;
	matrix R;
	matrix y;
	
	if (sonar_used)
	{
		matrix zk = matrix(2,1,baro, last_valid_sonar);
		H = matrix(2,5,
		1.0, 0.0, 0.0, 0.0, 0.0,
		x[4], 0.0, 0.0, -x[4], x[0]-x[3]);
		R = matrix::diag(2, compensate_land_effect ? 6000.0: 80.0, 5.0);
		y = matrix(2,1,baro-x[0], last_valid_sonar - x[4] * (x[0]-x[3]));		
		//y = zk - H * x;
	}
	else
	{
		y = matrix(1,1,baro-x[0]);
		H = matrix(1,5,
		1.0, 0.0, 0.0, 0.0, 0.0);
		R = matrix::diag(1, compensate_land_effect ? 6000.0: 80.0);
	}
	
	Q[12] = static_mode ? 1e-3 : 1e-7;

	matrix x1 = F * x + B * u;
	matrix P1 = F * P * F.transpos() + Q;
	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inverse();
	matrix sk_i = Sk.inverse();

	x = x1 + K*y;
	P = (matrix(P1.m) - K*H) * P1;

	TRACE("x[0-4]=%.3f,%.3f,%.3f,%.3f, accelz = %.3f, baro=%.3f, sonar=%.3f     \n", x[0], x[1], x[2], x[3], accelz, baro, sonar);
	log2(x.data, 6, 20);

	return 0;
}

