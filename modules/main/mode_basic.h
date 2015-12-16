#pragma once

#include "flight_mode.h"

class flight_mode_basic : IFlightMode
{
public:
	flight_mode_basic(){}
	~flight_mode_basic(){}

	virtual int setup();
	virtual int exit();
	virtual int loop(float dt);
};
