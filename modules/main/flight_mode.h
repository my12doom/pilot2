#pragma once

class IFlightMode
{
public:
	// setup all environment, resetting all needed controller
	// called before 
	// if mode controller returns a error (negative values), the caller (autopilot) is responsible for switching into backup flight mode.
	virtual int setup() = 0;

	// clear up
	// always called exiting or failed entering flight mode
	virtual int exit() = 0;

	// main loop, called in main loop
	virtual int loop(float dt) = 0;
};


// to implement a new flight mode:

// setup each controllers/variables needed in setup(), return 0 on successs, negativa value for failuere.

// run all position or other controllers and output attitude and throttle setpoint by calling
// yap.attitude_controller.set_euler_target() or yap.attitude_controller.update_target_from_stick();
// and set yap.throttle_result to wanted value.
//   you can use yap.alt_controller.update(rate); then yap.throttle_result = yap.alt_controller.get_result()
//   or call yap.default_alt_controlling() to use default alt holding behavior.

// do cleanup in exit();