#pragma once

#include "flight_mode.h"
class flight_mode_of_loiter : public IFlightMode
{
public:
        flight_mode_of_loiter(){}
	~flight_mode_of_loiter(){}

	virtual int setup();
	virtual int exit();
	virtual int loop(float dt);
};
