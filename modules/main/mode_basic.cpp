#include "mode_basic.h"
#include "pilot.h"

int flight_mode_basic::setup()
{
	return 0;
}

int flight_mode_basic::exit()
{
	return 0;
}
int flight_mode_basic::loop(float dt)
{
	// altitude: direct throttle control
	yap.throttle_result = yap.rc[2];

	// attitude
	// airborne or armed and throttle up
	bool after_unlock_action = yap.airborne || yap.rc[2] > 0.55f;
	if (after_unlock_action)	// airborne or armed and throttle up
	{
		float stick[3] = {yap.rc[0], yap.rc[1], yap.rc[3]};
		yap.attitude_controll.update_target_from_stick(stick, dt);
	}
	else
	{
		float euler_target[3] = {0,0, NAN};
		yap.attitude_controll.set_euler_target(euler_target);
	}

	return 0;
}
