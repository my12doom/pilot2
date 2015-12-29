#pragma once

#include "flight_mode.h"

class flight_mode_RTL : IFlightMode
{
public:
	flight_mode_RTL(){}
	~flight_mode_RTL(){}

	virtual int setup();
	virtual int exit();
	virtual int loop(float dt);

protected:
	enum
	{
		turn_around,
		rise,
		move,
		loiter,
		down,
	} stage;

	float home_pos_ne[2];
	float start_pos_ne[2];
	float yaw_target;
	float bearing;


	float turn_around_tick;
	float rising_tick;
	float move_tick;
	float loiter_tick;
};
