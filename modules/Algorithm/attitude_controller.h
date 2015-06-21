#pragma once

#include <stdint.h>
#include <math.h>


class attitude_controller
{
public:
	attitude_controller();
	~attitude_controller();

	// provide current copter state
	int provide_states(const float *alt, const float *attitude, float throttle_realized, int motor_state, bool airborne);

	// call one of these three to set attitude target
	int set_attitude_target_quaternion(const float *quaternion);
	int set_attitude_target(const float *euler);
	int set_attitude_target_from_stick(const float *stick);

	// update the controller
	// dt: time interval
	// user_rate: user desired climb rate, usually from stick.
	int update(float dt);

	// reset controller
	// call this if the controller has just been engaged
	int reset();

	// torque in body frame, axis: [0-2] -> [roll, pitch, yaw]
	// unit for roll/pitch/yaw: undefined!
	int get_result(float *out);

protected:
};
