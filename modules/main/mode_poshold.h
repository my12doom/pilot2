#pragma once

#include "flight_mode.h"

class flight_mode_poshold : public IFlightMode
{
public:
        flight_mode_poshold(){}
	~flight_mode_poshold(){}

	virtual bool is_ready();
	virtual int setup();
	virtual int exit();
	virtual int loop(float dt);
};
