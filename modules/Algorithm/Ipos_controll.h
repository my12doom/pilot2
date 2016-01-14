#pragma once

class pos_controller_base
{
public:
	virtual ~pos_controller_base(){}
	// reset position setpoints to current position and velocity, initialize controll loops and engage position controll.
	// must call provide_ahrs() and provide_pos() before resetting or error will occur.
	virtual int reset() = 0;

	// provide desired velocity in body frame (forward, right)
	virtual int set_desired_velocity(float *desired_velocity) = 0;

	// provide desired action from stick action (roll & pitch only, stick[0,1] = [roll, pitch]).
	virtual int set_desired_stick(float *stick) = 0;

	// call this periodly to update the controller
	virtual int update_controller(float dt) = 0;

	// get controll loop results, in body frame
	virtual int get_target_angles(float *target_angles) = 0;

	// set new setpoints in earth frame(north,east)
	// use this to achieve waypoint navigating
	// the controller should do procedures similar to reset() to get smooth transition
	virtual int set_setpoint(float *pos, bool reset = true) = 0;

	// provide the position controller with attitude and position infomation.
	virtual int provide_attitue_position(float *eulers, float *pos, float *velocity) = 0;
};
