#pragma once
#include "Ipos_controll.h"
#include <stdio.h>

enum poshold_state
{
	pos,
	override,
	braking,
};

class pos_controller_old : pos_controller_base
{
public:
	pos_controller_old();
	virtual ~pos_controller_old();

	virtual int reset();
	virtual int set_desired_velocity(float *desired_velocity);
	virtual int set_desired_stick(float *stick);
	virtual int update_controller(float dt);
	virtual int get_target_angles(float *target_angles);
	virtual int set_setpoint(float *pos, bool reset = true);
	virtual int provide_attitue_position(float *eulers, float *pos, float *velocity);

// private:
	int check_states();				// check and transfer states.
	int move_desire_pos(float dt);
	int pos_to_rate(float dt);
	int rate_to_accel(float dt);
	int accel_to_lean_angles();
	int lean_angle_to_accel(float *angles, float *accels);

	// states
	float setpoint[2];						// [north, east]
	float desired_velocity[2];				// [forward, right]
	float desired_velocity_earth[2];		// [north, east]
	float pos[2];							// [north, east]
	float velocity[2];						// [north, east]
	float eulers[3];						// [roll, pitch, yaw]

	// controll loop variables
	float target_velocity[2];				// [north, east]
	float last_target_velocity[2];			// [north, east]
	float target_accel[2];					// [north, east]
	float target_euler[2];					// [roll, pitch]
	float pid[2][3];						// [north, east][p, i, d]
	float ff[2];

	float sin_yaw;
	float cos_yaw;
	float min_braking_speed;
	float release_stick_timer;

#ifdef WIN32
	FILE * f;
	unsigned int tick;
#endif
	poshold_state state;
	float pilot_angle[2];
};
