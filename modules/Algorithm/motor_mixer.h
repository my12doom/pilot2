#pragma once

#include "actuator.h"
#include <HAL/Interface/IRCOUT.h>

class motor_mixer : public actuator
{
	motor_mixer(HAL::IRCOUT * pwmout);
	~motor_mixer();
	int set_pwm_range(int throttle_stop, int throttle_idle, int throttle_max);

	// arming and disarming sequence functions

	// arm the actuator, allowing it to output any power, and preventing stalls mid-air.
	virtual int arm();

	// query arm state
	//   return 0 if all motor is spinned up and ready to fly.
	//   negtive values for any error, and stops all motors.
	//   possitive valus during spooling up.
	virtual int arm_state();

	// shuts down all motors, and do active braking if available.
	virtual int disarm();

	// acturating functions

	// set the target torque and thrust in body frame
	virtual int set_target(const float *torque, const float throttle);

	// get actuated torque and thrust
	//	in normal circumstances it should be exactly same of target.
	//	in actuator limited circumstances it returns actuated torque and thrust
	virtual int get_actuated(float *torque, float *throttle);

	// return actuator limitation stats for each axis.
	virtual int get_state(float *states);

	// return actuator defined state, limitations and errors.
	virtual int get_state2(void *p, int maxsize);

protected:
	int16_t throttle_stop;
	int16_t throttle_idle;
	int16_t throttle_max;
	int motor_count;
	int arming_state;
	float result[16];
};
