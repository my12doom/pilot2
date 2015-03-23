#pragma once

#include "actuator.h"

class motor_mixer : public actuator
{
	motor_mixer();
	~motor_mixer();

	// set the target torque and thrust in body frame
	int set_target(const float *torque, const float *throttle);

	//
	int get_result(float *out);
	int get_actuated(float *torque, float *throttle);
	int get_state(void *p, int maxsize);

};