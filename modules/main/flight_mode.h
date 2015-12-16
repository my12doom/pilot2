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
