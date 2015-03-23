#pragma once

#include <stdint.h>

class actuator
{
public:
	actuator();
	~actuator();

	// set the target torque and thrust in body frame
	virtual int set_target(const float *torque, const float *throttle) = 0;

	// get controller result similar to traditional pwm channels..
	//  for fixed wing, arrays are[aileron, elevator, throttle, rudder, other channels]
	//	for multicopters, arrays are motors defined by motor configuration
	//	or user defined arrays for any other airframe..
	virtual int get_result(float *out, int count) = 0;

	// get actuated torque and thrust
	//	in normal circumstances it should be exactly same of target.
	//	in actuator limited circumstances it returns actuated torque and thrust
	virtual int get_actuated(float *torque, float *throttle) = 0;

	// return actuator defined state, limitations and errors.
	virtual int get_state(void *p, int maxsize) = 0;
protected:

};