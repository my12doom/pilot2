#include "attitude_controller.h"
#include <string.h>
#include <Protocol/common.h>
#include <utils/param.h>

// parameters
#define yaw_dead_band 0.08f
static param quadcopter_range[3] = 
{
	param("rngR", PI / 5),			// roll
	param("rngP", PI / 5),			// pitch
	param("rngY", PI / 8),			// yaw
};

static param pid_factor[3][4] = 			// pid_factor[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("rP1",0.60), param("rI1",0.80), param("rD1",0.03),param("rM1",PI)},
	{param("rP2",0.60), param("rI2",0.80), param("rD2",0.03),param("rM2",PI)},
	{param("rP3",1.75), param("rI3",0.25), param("rD3",0.01),param("rM3",PI)},
};
static param pid_factor2[3][4] = 			// pid_factor2[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("sP1", 6), param("sI1", 0), param("sD1", 0),param("sM1", PI/45)},
	{param("sP2", 6), param("sI2", 0), param("sD2", 0),param("sM2", PI/45)},
	{param("sP3", 8), param("sI3", 0), param("sD3", 0),param("sM3", PI/45)},
};

static param QUADCOPTER_ACRO_YAW_RATE("raty", PI);
static param QUADCOPTER_MAX_YAW_OFFSET("offy", PI/4);

attitude_controller::attitude_controller()
{
	euler_sp[0] = 0;
	euler_sp[1] = 0;
	euler_sp[2] = NAN;

	reset();
}
attitude_controller::~attitude_controller()
{

}

// provide current copter state
// parameters:
// attitude[0-2] : [roll, pitch, yaw] in euler mode, [q0~q3] in quaternion mode.
// body rate[0-2] : [roll, pitch, yaw] rate in body frame.
// motor state: a combination of motor_limit enum, or 0 if all motors are normal, the controller will stop integrating at any saturated axis
// airborne: the controller will not integrate on ground.
int attitude_controller::provide_states(const float *attitude, const float *body_rate, uint32_t motor_state, bool airborne)
{
	if (use_quaternion)
		memcpy(quaternion, attitude, sizeof(float)*4);
	else
		memcpy(euler, attitude, sizeof(float)*3);

	memcpy(this->body_rate, body_rate, sizeof(float)*3);
	this->motor_state = motor_state;
	this->airborne = airborne;

	if (isnan(euler_sp[2]))
		euler_sp[2] = euler[2];

	return 0;
}

// call one of these three to set attitude target
int attitude_controller::set_quaternion_target(const float *quaternion)
{
	if (!use_quaternion)
		return -1;		// TODO: set euler target properly

	memcpy(quaternion_sp, quaternion, sizeof(float)*4);
	return 0;
}

int attitude_controller::set_euler_target(const float *euler)
{
	if (use_quaternion)
		return -1;		// TODO: set quaternion target properly

	for(int i=0; i<3; i++)
		if (!isnan(euler[i]))
			euler_sp[i] = euler[i];
	
	return 0;
}

int attitude_controller::update_target_from_stick(const float *stick, float dt)
{
	// copy stick command into member variable only, since no dt available
	// setpoint will be updated in update();
	// update set point if stick command exists
	// caller can pass any of three axis NAN to disable updading of that axis. to update yaw stick only in optical flow mode for example
	if (!use_quaternion)
	{
		// roll & pitch
		for(int i=0; i<2; i++)
		{
			if (isnan(stick[i]))
				continue;

			float limit_l = euler_sp[i] - 2*PI * dt;
			float limit_r = euler_sp[i] + 2*PI * dt;
			euler_sp[i] = stick[i] * quadcopter_range[i] * (i==1?-1:1);	// pitch stick and coordinate are reversed 
			euler_sp[i] = limit(euler_sp[i], limit_l, limit_r);
		}
		
		// yaw
		if (!isnan(stick[2]))
		{
			float delta_yaw = ((fabs(stick[2]) < yaw_dead_band) ? 0 : stick[2]) * dt * QUADCOPTER_ACRO_YAW_RATE;
			float new_target = radian_add(euler_sp[2], delta_yaw);
			float old_error = abs(radian_sub(euler_sp[2], euler[2]));
			float new_error = abs(radian_sub(new_target, euler[2]));
			if (new_error < ((airborne&&!motor_state)?QUADCOPTER_MAX_YAW_OFFSET:(QUADCOPTER_MAX_YAW_OFFSET/5)) || new_error < old_error)	// decrease max allowed yaw offset if any motor saturated
				euler_sp[2] = new_target;
		}
	}
	else
	{
		// TODO: there is no rate limitation here.
		// TODO: there isn't even a implementation!
		
		return -1;
	}
	
	return 0;
}

// update the controller
// dt: time interval
int attitude_controller::update(float dt)
{

	// outter loop, attitude -> body frame rate
	if (use_quaternion)
	{
		// TODO: implemente it!
	}
	else
	{
		for(int i=0; i<3; i++)
		{
			body_rate_sp[i] = radian_sub(euler_sp[i], euler[i]) * pid_factor2[i][0];
			body_rate_sp[i] = limit(body_rate_sp[i], -PI, PI);
		}
	}
	
	
	// inner loop, body frame rate -> body frame torque.
	for(int i=0; i<3; i++)
	{
		float new_p = body_rate_sp[i] - body_rate[i];

		// I
		// TODO: handle axies independently.
		if (airborne)								// only integrate after takeoff
		if (!motor_state || new_p * pid[i][1] < 0)	// only integrate if no motor is saturated or integration will decrease.
		{
			pid[i][1] += new_p * dt;
			pid[i][1] = limit(pid[i][1], -pid_factor[i][3], pid_factor[i][3]);
		}

		// D, with 40hz low pass filter
		static const float lpf_RC = 1.0f/(2*PI * 40.0f);
		float alpha = dt / (dt + lpf_RC);
		float derivative = (new_p - pid[i][0] )/dt;
		pid[i][2] = pid[i][2] * (1-alpha) + derivative * alpha;

		// P
		pid[i][0] = new_p;

		// sum
		result[i] = 0;
		for(int j=0; j<3; j++)
			result[i] += pid[i][j] * pid_factor[i][j];
	}
	TRACE(", pid=%.2f, %.2f, %.2f\n", pid_result[0], pid_result[1], pid_result[2]);
	
	
	return 0;
}

// reset controller
// call this if the controller has just been engaged
int attitude_controller::reset()
{
	memcpy(euler_sp, euler, sizeof(euler));
	memcpy(body_rate_sp, body_rate_sp, sizeof(body_rate_sp));
	for(int i=0; i<3; i++)
		pid[i][1] = 0;

	return 0;
}

// torque in body frame, axis: [0-2] -> [roll, pitch, yaw]
// unit for roll/pitch/yaw: undefined!
int attitude_controller::get_result(float *out)
{
	memcpy(out, result, sizeof(float)*3);

	return 0;
}
