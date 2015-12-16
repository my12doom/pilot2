#pragma once

#include "flight_mode.h"

class flight_mode_althold : IFlightMode
{
public:
	flight_mode_althold(){}
	~flight_mode_althold(){}

	virtual int setup();
	virtual int exit();
	virtual int loop(float dt);
};
