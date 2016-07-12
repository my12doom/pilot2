#pragma once

#include <stdint.h>
#include <math.h>

class attitude_controller
{
public:
	attitude_controller();
	~attitude_controller();

	// provide current copter state
	// parameters:
	// euler[0-2] : [roll, pitch, yaw]
	// quaternion[0-3] : [q0~q3]
	// body rate[0-2] : [roll, pitch, yaw] rate in body frame.
	// motor state: a combination of motor_limit enum, or 0 if all motors are normal, the controller will stop integrating at any saturated axis
	// airborne: the controller will not integrate on ground.
	int provide_states(const float *euler, const float *quaternion, const float *body_rate, uint32_t motor_state, bool airborne);

	// call one of these three to set attitude target
	// pass NAN for any euler axis if you want it to remain unchanged
	// update_target_from_stick() can be used simultaneously to update one or more axis, usually yaw axis, pass NAN for axis you want to remain unchanged.
	int set_quaternion_target(const float *quaternion);
	int set_euler_target(const float *euler);
	int update_target_from_stick(const float *stick, float dt);

	// update the controller
	// dt: time interval
	int update(float dt);

	// reset controller
	// call this if the controller has just been engaged or quaternion mode setting changed.
	int reset();

	// torque in body frame, axis: [0-2] -> [roll, pitch, yaw]
	// unit for roll/pitch/yaw: undefined!
	int get_result(float *out);

	// toggle quaternion outter loop mode
	// a reset() is recommended after changing this settings.
	void set_quaternion_mode(bool use_quaternion){this->use_quaternion = use_quaternion;}

	// set body rate override for each axis.
	// usually this is used for acrobatic moves.
	// set to NAN to revert back to normal attitude controlling.
	int set_body_rate_override(const float *override);

	// helper function:
	// map rectangular stick to a rounded attitude
	static void get_attitude_from_stick(const float *stick, float *attitude);

//protected:
	float body_rate[3];
	float body_rate_sp[3];
	float body_rate_sp_override[3];		// body rate override, usually used for acrobatic moves;
	float euler[3];
	float euler_sp[3];		// sp = set point
	float last_set_euler_sp_time;
	float quaternion[4];
	float quaternion_sp[4];		// sp = set point
	bool airborne;
	uint32_t motor_state;
	bool use_quaternion;
	bool just_reseted;
	
	float pid[3][3];		// [roll, pitch, yaw] [p, i, d]
	float result[3];
};
