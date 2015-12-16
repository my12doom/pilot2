#include "mode_althold.h"
#include "pilot.h"

int flight_mode_althold::setup()
{
	yap.alt_controller.reset();
	
	return 0;
}
int flight_mode_althold::exit()
{
	return 0;
}
int flight_mode_althold::loop(float dt)
{
	// altitude
	yap.default_alt_controlling();

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
