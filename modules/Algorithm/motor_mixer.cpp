#include "motor_mixer.h"
#include <math.h>
#include <utils/param.h>
#include <protocol/common.h>

#define MAX_MOTOR_COUNT 8
#define QUADCOPTER_THROTTLE_RESERVE 0.15f
static param THROTTLE_IDLE("idle", 1176);
static param motor_matrix("mat", 0);

static float quadcopter_mixing_matrix[2][MAX_MOTOR_COUNT][3] = // the motor mixing matrix, [motor number] [roll, pitch, yaw]
{
	{							// + mode
		{0, -1, +1},			// rear, CCW
		{-1, 0, -1},			// right, CW
		{0, +1, +1},			// front, CCW
		{+1, 0, -1},			// left, CW
	},
	{							// X mode
		{-0.707f,-0.707f,+0.707f},				//REAR_R, CCW
		{-0.707f,+0.707f,-0.707f},				//FRONT_R, CW
		{+0.707f,+0.707f,+0.707f},				//FRONT_L, CCW
		{+0.707f,-0.707f,-0.707f},				//REAR_L, CW
	}
};

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}
static int16_t max(int16_t a, int16_t b)
{
	return a<b?b:a;
}


motor_mixer::motor_mixer(HAL::IRCOUT * pwmout)
{
	int matrix = (float)motor_matrix;
	for(int i=0; i<MAX_MOTOR_COUNT; i++)
	{
		if (quadcopter_mixing_matrix[matrix][i][0] == quadcopter_mixing_matrix[matrix][i][1] && 
			quadcopter_mixing_matrix[matrix][i][1] == quadcopter_mixing_matrix[matrix][i][2] && 
			fabs((float)quadcopter_mixing_matrix[matrix][i][2]) < 0.01f)
		{
			motor_count = i;
			break;			
		}
	}
}
motor_mixer::~motor_mixer()
{

}

int motor_mixer::set_pwm_range(int throttle_stop, int throttle_idle, int throttle_max)
{
	this->throttle_stop = throttle_stop;
	this->throttle_idle = throttle_idle;
	this->throttle_max = throttle_max;
	
	return 0;	
}


// arming and disarming sequence functions

// arm the actuator, allowing it to output any power, and preventing stalls mid-air.
int motor_mixer::arm()
{
	return -1;
}

// query arm state
//   return 0 if all motor is spinned up and ready to fly.
//   negtive values for any error, and stops all motors.
//   possitive valus during spooling up.
int motor_mixer::arm_state()
{
	return 0;
}

// shuts down all motors, and do active braking if available.
int motor_mixer::disarm()
{
	return 0;
}

// acturating functions

// set the target torque and thrust in body frame
int motor_mixer::set_target(const float *torque, const float throttle)
{
	float throttle_actuated = 0;
	int matrix = (float)motor_matrix;
	int motor_count = MAX_MOTOR_COUNT;

	// mixing
	for(int motor=0; motor<motor_count; motor++)
	{
		result[motor] = 0;

		for(int j=0; j<3; j++)
			result[motor] += quadcopter_mixing_matrix[matrix][motor][j] * torque[j] * QUADCOPTER_THROTTLE_RESERVE;

		result[motor] = limit(throttle_idle + result[motor]*(throttle_max-throttle_idle), throttle_idle, throttle_max);

		throttle_actuated += result[motor];
	}
	
	// roll & pitch saturation?
	// throttle saturation?
	// apply scale for roll & pitch to make it unsaturated.


	// yaw..
	
	// calculate actuated torque and throttle
	throttle_actuated /= motor_count;
	
	return 0;
}

// get actuated torque and thrust
//	in normal circumstances it should be exactly same of target.
//	in actuator limited circumstances it returns actuated torque and thrust
int motor_mixer::get_actuated(float *torque, float *throttle)
{
	return 0;
}

// return actuator limitation stats for each axis.
int motor_mixer::get_state(float *states)
{
	return 0;
}

// return actuator defined state, limitations and errors.
int motor_mixer::get_state2(void *p, int maxsize)
{
	return 0;
}

