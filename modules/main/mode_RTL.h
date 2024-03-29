#pragma once

#include "flight_mode.h"


typedef struct  
{
	float pos[2];
	float pos_setpoint[2];
	float euler_setpoint[3];
	float baro_setpoint;
	float sonar_setpoint;
} RTL_state;

class flight_mode_RTL : public IFlightMode
{
public:
	flight_mode_RTL(){}
	~flight_mode_RTL(){}

	virtual bool is_ready();
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

	RTL_state s;
	bool islanding_before;
	bool light_before;
};
