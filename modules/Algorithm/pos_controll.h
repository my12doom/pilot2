#pragma once
#include <stdio.h>

class pos_controller_base
{
public:
	virtual ~pos_controller_base(){}
	// reset position setpoints to current position and velocity, initialize controll loops and engage position controll.
	// must call provide_ahrs() and provide_pos() before resetting or error will occur.
	virtual int reset() = 0;

	// provide desired velocity in body frame (forward, right)
	virtual int set_desired_velocity(float *desired_velocity) = 0;

	// call this periodly to update the controller
	virtual int update_controller(float dt) = 0;

	// get controll loop results, in body frame
	virtual int get_target_angles(float *target_angles) = 0;

	// set new setpoints in earth frame(north,east)
	// use this to achieve waypoint navigating
	// the controller should do procedures similar to reset() to get smooth transition
	virtual int set_setpoint(float *pos) = 0;

	// provide the position controller with attitude and position infomation.
	virtual int provide_attitue_position(float *eulers, float *pos, float *velocity) = 0;
};

class pos_controller : pos_controller_base
{
public:
	pos_controller();
	virtual ~pos_controller();

	virtual int reset();
	virtual int set_desired_velocity(float *desired_velocity);
	virtual int update_controller(float dt);
	virtual int get_target_angles(float *target_angles);
	virtual int set_setpoint(float *pos);
	virtual int provide_attitue_position(float *eulers, float *pos, float *velocity);

// private:
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

#ifdef WIN32
	FILE * f;
	unsigned int tick;
#endif
};
