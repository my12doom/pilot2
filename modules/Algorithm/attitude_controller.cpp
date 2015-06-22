#include "attitude_controller.h"
#include <string.h>
#include <Protocol/common.h>
#include <utils/param.h>

static param quadcopter_range[3] = 
{
	param("rngR", PI / 5),			// roll
	param("rngP", PI / 5),			// pitch
	param("rngY", PI / 8),			// yaw
};

attitude_controller::attitude_controller()
{
	reset();
}
attitude_controller::~attitude_controller()
{

}

// provide current copter state
// parameters:
// attitude[0-2] : [roll, pitch, yaw, (body frame) roll rate, pitch rate, yaw rate, (optimal if quaternion attitude control not used)q0, q1, q2, q3]
// motor state: a combination of motor_limit enum, or 0 if all motors are normal
int attitude_controller::provide_states(const float *attitude, const float *body_rate, uint32_t motor_state, bool airborne)
{
	return 0;
}

// call one of these three to set attitude target
int attitude_controller::set_quaternion_target(const float *quaternion)
{
	if (!use_quaternion)
		return -1;		// TODO: set euler target properly

	stick[0] = NAN;		// clear stick command
	memcpy(quaternion_sp, quaternion, sizeof(float)*4);
	return 0;
}

int attitude_controller::set_euler_target(const float *euler)
{
	if (quaternion)
		return -1;		// TODO: set quaternion target properly

	stick[0] = NAN;		// clear stick command
	memcpy(euler_sp, euler, sizeof(float)*3);
	return 0;
}

int attitude_controller::set_stick_target(const float *stick)
{
	// copy stick command into member variable only, since no dt available
	// setpoint will be updated in update();
	memcpy(this->stick, stick, sizeof(float)*3);
	return 0;
}

// update the controller
// dt: time interval
// user_rate: user desired climb rate, usually from stick.
int attitude_controller::update(float dt)
{
	// update set point if stick command exists
	if (!isnan(stick[0]))
	{
		if (!use_quaternion)
		{
			// roll & pitch
			for(int i=0; i<2; i++)
			{
				float limit_l = euler[i] - PI*2 * dt;
				float limit_r = euler[i] + PI*2 * dt;
				euler_sp[i] = stick[i] * quadcopter_range[i] * (i==1?-1:1);	// pitch stick and coordinate are reversed 
				euler_sp[i] = limit(euler_sp[i], limit_l, limit_r);
			}
			
			// yaw
		}
		else
		{
			// TODO: there is no rate limitation here.
			// TODO: there isn't even a implementation!
			
			return -1;
		}
	}
	
	// outter loop, attitude -> body frame rate
	
	
	// inner loop, body frame rate -> body frame torque.
	
	
	return 0;
}

// reset controller
// call this if the controller has just been engaged
int attitude_controller::reset()
{
	stick[0] = NAN;

}

// torque in body frame, axis: [0-2] -> [roll, pitch, yaw]
// unit for roll/pitch/yaw: undefined!
int attitude_controller::get_result(float *out)
{
	memcpy(out, result, sizeof(float)*3);

	return 0;
}
