#include "attitude_controller.h"
#include <string.h>
#include <Protocol/common.h>
#include <utils/param.h>
#include <math/quaternion.h>
#include <HAL/Interface/ISysTimer.h>
#include <utils/log.h>

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

static param rate_trim[3] = 
{
	param("triR", 0),
	param("triP", 0),
	param("triY", 0),
};

static param QUADCOPTER_ACRO_YAW_RATE("raty", PI);
static param QUADCOPTER_MAX_YAW_OFFSET("offy", PI/8);
static param slew_rate("slew", PI*2);

attitude_controller::attitude_controller()
{
        euler_sp[0] = 0;
	euler_sp[1] = 0;
	euler_sp[2] = NAN;
	last_set_euler_sp_time = 0;
	just_reseted = true;
	for(int i=0; i<3; i++)
		body_rate_sp_override[i] = NAN;
	reset();
}
attitude_controller::~attitude_controller()
{

}

// provide current copter state
// parameters:
// euler[0-2] : [roll, pitch, yaw]
// quaternion[0-3] : [q0~q3]
// body rate[0-2] : [roll, pitch, yaw] rate in body frame.
// motor state: a combination of motor_limit enum, or 0 if all motors are normal, the controller will stop integrating at any saturated axis
// airborne: the controller will not integrate on ground.
int attitude_controller::provide_states(const float *euler, const float *quaternion, const float *body_rate, uint32_t motor_state, bool airborne)
{
	if (euler)
		memcpy(this->euler, euler, sizeof(float)*3);
	if (quaternion)
		memcpy(this->quaternion, quaternion, sizeof(float)*4);

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

int attitude_controller::set_body_rate_override(const float *override)
{
	if (!override)
		return -1;

	memcpy(body_rate_sp_override, override, sizeof(body_rate_sp_override));

	return 0;
}

int attitude_controller::set_euler_target(const float *euler)
{
	// smooth out minor high frequency changes from position controllers
	float dt = limit((systimer->gettime() - last_set_euler_sp_time)/1000000.0f, 0, 1);
	float hz = 10.0f;
	float threshold = 2.0f * PI / 180;
	bool go = false;
	for(int i=0; i<3; i++)
		if (!isnan(euler[i]) && (dt > 1.0f/hz || fabs(euler[i] - euler_sp[i]) > threshold))
			go = true;

	for(int i=0; i<3; i++)
	{
		if (!isnan(euler[i]) && go)
		{
			euler_sp[i] = euler[i];
			last_set_euler_sp_time = systimer->gettime();
		}
	}


	return 0;
}

static inline float fmin(float a, float b)
{
	return a<b?a:b;
}


// helper function:
// map rectangular stick to a rounded attitude
void attitude_controller::get_attitude_from_stick(const float *stick, float *attitude)
{
	attitude[0] = stick[0] * quadcopter_range[0];
	attitude[1] = -stick[1] * quadcopter_range[1];

	if (fabs(stick[0]) > 0.001f && fabs(stick[1]) > 0.001f)
	{
		float v = fmin(fabs(stick[0]/stick[1]), fabs(stick[1]/stick[0]));
		float f = 1.0f / sqrt(1+v*v);

		attitude[0] *= f;
		attitude[1] *= f;
	}

}

int attitude_controller::update_target_from_stick(const float *stick, float dt)
{
	// copy stick command into member variable only, since no dt available
	// setpoint will be updated in update();
	// update set point if stick command exists
	// caller can pass any of three axis NAN to disable updading of that axis. to update yaw stick only in optical flow mode for example

	// roll & pitch
	if (!isnan(stick[0]) && !isnan(stick[1]))
	{
		float att[2];
		get_attitude_from_stick(stick, att);

		for(int i=0; i<2; i++)
		{
			float limit_l = euler_sp[i] - slew_rate * dt;
			float limit_r = euler_sp[i] + slew_rate * dt;
			euler_sp[i] = limit(att[i], limit_l, limit_r);
		}
	}
	
	// yaw
	if (!isnan(stick[2]))
	{
		float delta_yaw = ((fabs(stick[2]) < yaw_dead_band) ? 0 : stick[2]) * dt * QUADCOPTER_ACRO_YAW_RATE;
		float new_target = radian_add(euler_sp[2], delta_yaw);
		float old_error = fabs(radian_sub(euler_sp[2], euler[2]));
		float new_error = fabs(radian_sub(new_target, euler[2]));
		if (new_error < ((airborne&&!motor_state)?QUADCOPTER_MAX_YAW_OFFSET:(QUADCOPTER_MAX_YAW_OFFSET/5)) || new_error < old_error)	// decrease max allowed yaw offset if any motor saturated
			euler_sp[2] = new_target;
	}

	return 0;
}

// update the controller
// dt: time interval
int attitude_controller::update(float dt)
{
	// limit max yaw set point
	float delta_angle = radian_sub(euler_sp[2], euler[2]);
	delta_angle = limit(delta_angle, -QUADCOPTER_MAX_YAW_OFFSET, QUADCOPTER_MAX_YAW_OFFSET);
	euler_sp[2] = radian_add(euler[2], delta_angle);

	// outter loop, attitude -> body frame rate
	if (use_quaternion)
	{
		float q_desired[4];
		float q_error[4];
		float body_frame_error[3];
		RPY2Quaternion(euler_sp, q_desired);
		quat_inverse(q_desired);
		quat_mult(q_desired, quaternion, q_error);
		quat_inverse(q_error);
		Quaternion2BFAngle(q_error, body_frame_error);
		for(int i=0; i<3; i++)
			body_rate_sp[i] = limit(body_frame_error[i] * pid_factor2[i][0], -PI, PI);

		//printf("br_sp:%.1f,%.1f,%.1f\n", body_rate_sp[0]*180/PI, body_rate_sp[1]*180/PI, body_rate_sp[2]*180/PI);

		//printf("Q:%f, %.3f,%.3f,%.3f,%.3f\n", body_rate_sp[0], quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
	}
	else
	{
		for(int i=0; i<3; i++)
		{
			body_rate_sp[i] = radian_sub(euler_sp[i], euler[i]) * pid_factor2[i][0];
			body_rate_sp[i] = limit(body_rate_sp[i], -PI, PI);
		}
	}

	// body rate override
	for(int i=0; i<3; i++)
	{
		if (!isnan(body_rate_sp_override[i]))
			body_rate_sp[i] = body_rate_sp_override[i];
	}

	//printf("euler_sp:%.1f, rate:%.1f/%.1f\n", euler_sp[0]*180/PI, body_rate[0]*180/PI, body_rate_sp[0]*180/PI);

	//float dbg[4] = {euler_sp[1], body_rate_sp[1], euler[1], body_rate[1]};
	//log2(dbg, TAG_ATTITUDE_CONTROLLER_DATA, sizeof(dbg));
	
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

		// D, with 10hz low pass filter
		static const float lpf_RC = 1.0f/(2*PI * 20.0f);
		float alpha = dt / (dt + lpf_RC);
		float derivative = (new_p - pid[i][0] )/dt;
		if (just_reseted)
			derivative = 0;
		pid[i][2] = pid[i][2] * (1-alpha) + derivative * alpha;

		// P
		pid[i][0] = new_p;

		// sum
		result[i] = 0;
		for(int j=0; j<(airborne?3:2); j++)
			result[i] += pid[i][j] * pid_factor[i][j];
	}

	just_reseted = false;
	//printf(", pid=%.2f, %.2f, %.2f\n", result[0], result[1], result[2]);
	
	
	return 0;
}

// reset controller
// call this if the controller has just been engaged
int attitude_controller::reset()
{
    memcpy(euler_sp, euler, sizeof(euler));
    memcpy(body_rate_sp, body_rate, sizeof(body_rate_sp));
    for(int i=0; i<3; i++)
    {
        pid[i][1] = rate_trim[i];
        pid[i][0] = 0;
        pid[i][2] = 0;
    }
    just_reseted = true;

    return 0;
}

// torque in body frame, axis: [0-2] -> [roll, pitch, yaw]
// unit for roll/pitch/yaw: undefined!
int attitude_controller::get_result(float *out)
{
	memcpy(out, result, sizeof(float)*3);

	return 0;
}
