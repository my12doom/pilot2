#pragma once

#include "actuator.h"

class motor_mixer : public actuator
{
	motor_mixer();
	~motor_mixer();

	// set the target torque and thrust in body frame
	virtual int set_target(const float *torque, const float *throttle);

	//
	virtual int get_result(float *out, int count);
	virtual int get_actuated(float *torque, float *throttle);
	virtual int get_state(float *states) = 0;
	virtual int get_state2(void *p, int maxsize);

};