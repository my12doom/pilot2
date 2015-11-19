#include "battery_estimator.h"
#include <HAL/Interface/Interfaces.h>
#include <protocol/common.h>

static const matrix Q
(
	2,2,
	4e-11, 0.0,
	0.0, 4e-15
);
static const matrix R
(
1,1,
0.01*0.01
);

float total_voltage_span = 12.6f-10.8f;
float total_mah = 1800;
float initial_registance_change_rate = 0.010f / 700;

battery_estimator::battery_estimator()
:last_log(0)
,init(false)
,mah(0)
,P(2,2,200.0, 0.0, 0.0, 200.0)
{

}
battery_estimator::~battery_estimator()
{
	
}


int battery_estimator::update(const float voltage, const float current, const float dt)
{
	matrix F
	(
		2,2,
		1.0, 0.0,
		0.0, 1.0
	);
	matrix H
	(1,2,
	1.0, -current
	);
	matrix zk
	(1,1,
	voltage
	);

	matrix B
		(2,1,
		-total_voltage_span / total_mah * dt / 3.6f,
		0.0
		);

	matrix u(1,1,current);
	matrix G
	(
		2,2,
		-total_voltage_span / total_mah * dt / 3.6f, 0.0,
		0.0, dt
	);

	if (!init)
	{
		x.m = 2;
		x.n = 1;
		x[1] = 0.040f;//	40mOhm
		x[0] = voltage + current * 0.040f;

		init = true;
	}

	matrix x1 = F * x + B * u;
	matrix P1 = F * P * F.transpos() + Q;
	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inverse();
	matrix sk_i = Sk.inverse();

	x = x1 + K*(zk - H*x1);
	P = (matrix(P1.m) - K*H) * P1;
	
	if (systimer->gettime() > last_log + 2000000)
	{
		TRACE("Batt:%.3f/%.3fV, %d mOhm", voltage, x.data[0], int(x.data[1]*1000));
		last_log = systimer->gettime();
	}

	return 0;
}
