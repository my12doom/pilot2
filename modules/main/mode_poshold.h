#pragma once

#include "flight_mode.h"

class flight_mode_poshold : IFlightMode
{
public:
	flight_mode_poshold(){}
	~flight_mode_poshold(){}

	virtual int setup();
	virtual int exit();
	virtual int loop(float dt);
};
