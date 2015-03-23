#pragma once

#include <stdint.h>
#include <math.h>


class attitude_controller
{
public:
	attitude_controller();
	~attitude_controller();

	int provide_states(float *alt, float *attitude, float throttle_realized, int motor_state, bool airborne);

	int set_attitude_target(float *euler);
	int set_attitude_target_from_stick()

	// update the controller
	// dt: time interval
	// user_rate: user desired climb rate, usually from stick.
	int update(float dt);

	// reset controller
	// call this if the controller has just been engaged
	int reset();

	// torque in body frame, axis: [0-2] -> [roll, pitch, yaw]
	int get_result(float *out);

protected:
};
