#pragma once

#include <stdint.h>

class actuator
{
public:
	// arming and disarming sequence functions

	// arm the actuator, allowing it to output any power, and preventing stalls mid-air.
	virtual int arm() = 0;

	// query arm state
	//   return 0 if all motor is spinned up and ready to fly.
	//   negtive values for any error, and stops all motors.
	//   possitive valus during spooling up.
	virtual int arm_state() = 0;

	// shuts down all motors, and do active braking if available.
	virtual int disarm() = 0;

	// acturating functions

	// set the target torque and thrust in body frame
	//    outputting one shot or continuous is OK, since this function is called every main loop cycle.
	virtual int set_target(const float *torque, const float throttle) = 0;

	// get actuated torque and thrust
	//	in normal circumstances it should be exactly same of target.
	//	in actuator limited circumstances it returns actuated torque and thrust
	virtual int get_actuated(float *torque, float *throttle) = 0;

	// return actuator limitation stats for each axis.
	virtual int get_state(float *states) = 0;

	// return actuator defined state, limitations and errors.
	virtual int get_state2(void *p, int maxsize) = 0;
};
