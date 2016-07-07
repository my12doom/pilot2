#include "pos_controll.h"
#include <string.h>
#include <math.h>
#include <float.h>
#include <Protocol/common.h>
#include <utils/param.h>
#include <utils/log.h>

// constants
// static float leash = 5.0f;
static float linear_distance = 5.0f;
#ifndef WIN32
static param max_speed("maxH", 5.0f);
#else
static float max_speed = 5.0f;
#endif
static float max_speed_ff = 2.0f;
static bool use_desired_feed_forward = false;
static float feed_forward_factor = 1;
static float rate2accel[4] = {1.2f, 0.5f, 0.2f, 6.0f};
static float pos2rate_P = 1.0f;

// "parameters/constants"
#ifndef WIN32
static param quadcopter_range[3] = 
{
	param("rngR", PI / 8),			// roll
	param("rngP", PI / 8),			// pitch
	param("rngY", PI / 8),			// yaw
};
#else
static float quadcopter_range[3] = {PI/4, PI/4, PI/4};
#endif
// win32 helper
#ifdef WIN32
#include <Windows.h>
#define isnan _isnan
static unsigned long pnan[2]={0xffffffff, 0x7fffffff};
static double NAN = *( double* )pnan;
#endif

// helper functions
static float length(float a, float b)
{
	return sqrt(a*a+b*b);
}

static float length(float a, float b, float c)
{
	return sqrt(a*a+b*b+c*c);
}
static inline float fmin(float a, float b)
{
	return a<b?a:b;
}

static float sqrt2(float in)
{
	float o = in>0?1:-1;
	return o*sqrt(fabs(in));
}

pos_controller::pos_controller()
{
	release_stick_tick = 0;
	low_speed_tick = 0;
#ifdef WIN32
	f = fopen("Z:\\log.csv", "wb");
	fprintf(f, "time,v,tv,p,i,roll_target,pitch_target\r\n");
	tick = GetTickCount();

	float v = atan2(1, G_in_ms2);
	printf("%f\n", v * 180 / PI);
#endif
}

pos_controller::~pos_controller()
{

}

int pos_controller::reset()
{
	set_setpoint(pos);

	target_euler[0] = eulers[0];
	target_euler[1] = eulers[1];

	state = direct;
	low_speed_tick = 0;
	limit_accel = false;
	limit_angle = false;

	return 0;
}

int pos_controller::set_desired_velocity(float *desired_velocity)
{
	memcpy(this->desired_velocity, desired_velocity, sizeof(this->desired_velocity));

	// rotate from body frame to earth frame
	desired_velocity_earth[0] = cos_yaw * desired_velocity[0] - sin_yaw * desired_velocity[1];
	desired_velocity_earth[1] = sin_yaw * desired_velocity[0] + cos_yaw * desired_velocity[1];

	pilot_stick[0] = NAN;
	pilot_stick[1] = NAN;

	return 0;
}

int pos_controller::set_desired_stick(float *stick)
{
	float desired_velocity[2] = {stick[1] * max_speed, stick[0] * max_speed};
	if (fabs(desired_velocity[0]) < 0.4f)
		desired_velocity[0] = 0;
	if (fabs(desired_velocity[1]) < 0.4f)
		desired_velocity[1] = 0;

	if (state == loiter)
	{
		// copied from set_desired_velocity()
		desired_velocity_earth[0] = cos_yaw * desired_velocity[0] - sin_yaw * desired_velocity[1];
		desired_velocity_earth[1] = sin_yaw * desired_velocity[0] + cos_yaw * desired_velocity[1];
	}

	pilot_stick[0] = stick[0];
	pilot_stick[1] = stick[1];

	return 0;
}

int pos_controller::update_state_machine(float dt)
{
	poshold_state next_state = state;

	// a pair of Schmidt values used
	float threshold = 30.0f/500;
	if (state == loiter)
		threshold *= 1.5f;
	bool released = (isnan(pilot_stick[0]) || isnan(pilot_stick[1])) || (fabs(pilot_stick[0]) < threshold && fabs(pilot_stick[1]) < threshold);

	switch(state)
	{
	case direct:
		{
			// user released stick for a while(0.2s currently)
			if (released)
			{
				release_stick_tick += dt;
				if (release_stick_tick > 0.2f)
				{
					LOGE("braking start\n");
					next_state = braking;
				}
			}
			else
			{
				release_stick_tick = 0;
			}
		}
		break;

	case braking:
		{
			if (!released)		// if user moved stick again
			{
				LOGE("abort braking, direct\n");
				next_state = direct;
			}

			float speed = sqrt(velocity[0]*velocity[0] + velocity[1] * velocity[1]);

			if (speed < 1.0f)
				low_speed_tick += dt;
			else
				low_speed_tick = 0;

			if (low_speed_tick > 0.3f)		// speed low enough for a while ( 0.3s currently)
			{
				// update new setpoint
				setpoint[0] = pos[0] + velocity[0] / pos2rate_P * 0.25f;
				setpoint[1] = pos[1] + velocity[1] / pos2rate_P * 0.25f;

				LOGE("braking done, new setpoint to %.2f, %.2f\n", setpoint[0], setpoint[1]);

				next_state = loiter;
			}
		}
		break;

	case loiter:
		{
			if (!released)		// the user moved stick ?
			{
				next_state = direct;
			}
		}
		break;
	}

	if (next_state != state)
	{
		// reset tick
		low_speed_tick = 0;
		release_stick_tick = 0;

		// reset pid and feed forward
// 		memset(pid, 0, sizeof(pid));
		pid[0][0] = NAN;
		pid[1][0] = NAN;
		pid[0][2] = NAN;
		pid[1][2] = NAN;
		last_target_velocity[0] = NAN;
		ff[0] = 0;
		ff[1] = 0;
	}

	state = next_state;

	return 0;
}

int pos_controller::update_controller(float dt)
{
	update_state_machine(dt);

	if (state == loiter)
	{
		move_desire_pos(dt);

		pos_to_rate(dt);
	}

	else if (state == braking)
	{
		target_velocity[0] = 0;
		target_velocity[1] = 0;
	}

	if (state == loiter || state == braking)
	{
		rate_to_accel(dt);

		accel_to_lean_angles(dt);
	}
	else if (state == direct)
	{
		target_euler[0] = pilot_stick[0] * quadcopter_range[0];
		target_euler[1] = -pilot_stick[1] * quadcopter_range[1];

		// reduce I slowly during direct mode
		const float time_constant = 5.0f;
		float alpha = 1-dt/(dt+time_constant);
		pid[0][1] *= alpha;
		pid[1][1] *= alpha;
		
		// to accel
		float accel_forward = - tan(target_euler[1]) * G_in_ms2;
		float accel_right = tan(target_euler[0]) * G_in_ms2 / cos(eulers[1]);

		// rotate to earth frame
		float accel_north = accel_forward * cos_yaw - accel_right * sin_yaw;
		float accel_east = accel_forward * sin_yaw + accel_right * cos_yaw;

		// add I
		accel_north += pid[0][1] * rate2accel[1];
		accel_east += pid[1][1] * rate2accel[1];

		// rotate back to body frame
		accel_forward = accel_north * cos_yaw + accel_east * sin_yaw;
		accel_right = -accel_north * sin_yaw + accel_east * cos_yaw; 

		// back to angle and handle limitation
		// accel to lean angle
		float new_euler_pitch  = atan2(-accel_forward, G_in_ms2);
		float new_euler_roll = atan2(accel_right*cos(eulers[1]), G_in_ms2);		// maybe target_pitch not needed?

		if (fabs(new_euler_roll) > quadcopter_range[0] || fabs(new_euler_pitch) > quadcopter_range[1])
		{
			// TODO: handle angle limitation correctly
			float factor = fmin(quadcopter_range[0] / fabs(new_euler_roll) , quadcopter_range[1] / fabs(new_euler_pitch));
			new_euler_roll *= factor;
			new_euler_pitch *= factor;
		}

		target_euler[0] = new_euler_roll;
		target_euler[1] = new_euler_pitch;

		// reduce I slowly


	}
	else
	{
		LOGE("pos_controll: invalid state, switching to direct\n");
		state = direct;
	}

#ifdef WIN32
	fprintf(f, "%.3f,%f,%f,%f,%f,%.2f,%.2f\r\n", (GetTickCount()-tick)/1000.0f, velocity[0],target_velocity[0], pid[0][0], pid[0][1], target_euler[0] * 180 / PI, target_euler[1] * 180 / PI);
	fflush(f);
#else
	float logs[16] = {pos[0], pos[1], velocity[0], velocity[1], setpoint[0], setpoint[1], target_velocity[0], target_velocity[1], target_accel[0], target_accel[1], pid[0][0], pid[0][1], pid[0][2], pid[1][0], pid[1][1], pid[1][2]};
	log2(logs, TAG_POSC_DATA, sizeof(logs));

#endif


	return 0;
}

int pos_controller::get_target_angles(float *target_angles)
{
	memcpy(target_angles, this->target_euler, sizeof(this->target_euler));

	return 0;
}

int pos_controller::set_setpoint(float *pos, bool reset /*= true*/)
{
	float distance_ne[2] = {setpoint[0] - pos[0], setpoint[1] - pos[1]};
	float distance = sqrt(distance_ne[0]*distance_ne[0] + distance_ne[1] * distance_ne[1]);

	// reset pid if new setpoint is too far away from current setpoint, or forced reset.
	if (reset || distance > max_speed)
	{
		memset(pid, 0, sizeof(pid));
		pid[0][0] = NAN;
		pid[1][0] = NAN;
		pid[0][2] = NAN;
		pid[1][2] = NAN;
		last_target_velocity[0] = NAN;
		ff[0] = 0;
		ff[1] = 0;

		// lean angle to accel
		float accel_target[2];		// [forward, right]
		accel_target[0] = G_in_ms2 * (-sin(eulers[1])/cos(eulers[1]));		// lean forward = negetive pitch angle
		accel_target[1] = G_in_ms2 * (sin(eulers[0])/cos(eulers[0]));

		// rotate accel from forward-right to north-east axis
		float accel_north = cos_yaw * accel_target[0] - sin_yaw * accel_target[1];
		float accel_east = sin_yaw * accel_target[0] + cos_yaw * accel_target[1];

	}

	// set desired pos
	setpoint[0] = pos[0];
	setpoint[1] = pos[1];

	// TODO : do other checking and initializing
	state = loiter;

	return 0;
}

int pos_controller::provide_attitue_position(float *eulers, float *pos, float *velocity)
{
	memcpy(this->eulers, eulers, sizeof(this->eulers));
	memcpy(this->pos, pos, sizeof(this->pos));
	memcpy(this->velocity, velocity, sizeof(this->velocity));

	sin_yaw = sin(eulers[2]);
	cos_yaw = cos(eulers[2]);

	return 0;
}

int pos_controller::move_desire_pos(float dt)
{
	// move
	float new_setpoint[2] = 
	{
		setpoint[0] + desired_velocity_earth[0] * dt,
		setpoint[1] + desired_velocity_earth[1] * dt,
	};

	// limit only if with in leash distance or distance reduces.
	float new_distance = length(new_setpoint[0]-pos[0], new_setpoint[1] - pos[1]);
	float old_distance = length(setpoint[0]-pos[0], setpoint[1] - pos[1]);

	float leash = max_speed * 1.2f;
	if ( new_distance < leash || new_distance < old_distance)
	{
		setpoint[0] = new_setpoint[0];
		setpoint[1] = new_setpoint[1];
	}

	return 0;
}

int pos_controller::pos_to_rate(float dt)
{
	float error[2] = {setpoint[0] - pos[0], setpoint[1] - pos[1]};
// 	error[0] = sqrt2(error[0]);
// 	error[1] = sqrt2(error[1]);
	float distance_to_target = length(error[0], error[1]);

	// limit distance target
	float leash = max_speed;
	if (distance_to_target > leash)
	{
		error[0] *= leash / distance_to_target;
		error[1] *= leash / distance_to_target;
		distance_to_target = leash;
	}

	// use a non-linear P controller to get target velocity from position error.
	if (distance_to_target < linear_distance)
	{
		// target velocity grows with square root of distance.
		target_velocity[0] = error[0] * pos2rate_P;
		target_velocity[1] = error[1] * pos2rate_P;
	}
	else
	{
		// target velocity grows linearly with the distance
		// TODO: adjust numbers to make curve continuous
		//target_velocity[0] = sqrt(error[0]) * pos2rate_P;
		//target_velocity[1] = sqrt(error[1]) * pos2rate_P;
		float vel_sqrt = sqrt(distance_to_target) * sqrt(linear_distance);
		target_velocity[0] = error[0] * vel_sqrt / distance_to_target;
		target_velocity[1] = error[1] * vel_sqrt / distance_to_target;

	}

	// limit: limit to max speed if user input feed forward is not used.
	//		  limit to 2m/s if user input feed forward is used.
	float max_velocity_from_error = use_desired_feed_forward ? max_speed_ff : max_speed;
	float target_velocity_total = length(target_velocity[0], target_velocity[1]);
	if (target_velocity_total > max_velocity_from_error)
	{
		target_velocity[0] *= max_velocity_from_error / target_velocity_total;
		target_velocity[1] *= max_velocity_from_error / target_velocity_total;
	}

	if (use_desired_feed_forward)
	{
// 		target_velocity[0] *= (1-feed_forward_factor);
// 		target_velocity[1] *= (1-feed_forward_factor);
		target_velocity[0] += feed_forward_factor * desired_velocity_earth[0];
		target_velocity[1] += feed_forward_factor * desired_velocity_earth[1];
	}

	if (isnan(target_velocity[0]))
		printf("");

	return 0;
}

int pos_controller::rate_to_accel(float dt)
{
	float alpha5 = dt / (dt + 1.0f/(2*PI * 5.5f));
	float alpha2 = dt / (dt + 1.0f/(2*PI * 5.5f));

	if (!isnan(last_target_velocity[0]))
	{
		float ff0 = (target_velocity[0] - last_target_velocity[0]) / dt;
		float ff1 = (target_velocity[1] - last_target_velocity[1]) / dt;

		ff[0] = ff[0] * (1-alpha5) + ff0 * alpha5;
		ff[1] = ff[1] * (1-alpha5) + ff1 * alpha5;
	}
	last_target_velocity[0] = target_velocity[0];
	last_target_velocity[1] = target_velocity[1];

	// 2hz LPF for D term
	float alpha30 = dt / (dt + 1.0f/(2*PI * 2.0f));

	for(int axis=0; axis<2; axis++)
	{
		float p = target_velocity[axis] - velocity[axis];

		// apply a 5hz LPF to P
		if (isnan(pid[axis][0]))
			pid[axis][0] = p;
		else
			p = alpha2 * p + (1-alpha2) * pid[axis][0];

		float d = (p - pid[axis][0]) / dt;

		pid[axis][2] = isnan(pid[axis][2]) ? 0 : (pid[0][2] * (1-alpha30) + d * alpha30);
		pid[axis][0] = p;

		// update I only if we did not hit accel/angle limit or throttle limit or I term will reduce
		if (!limit_accel && !limit_angle && state != braking)
		{
			pid[axis][1] += p *  dt;
			pid[axis][1] = limit(pid[axis][1], -rate2accel[3], rate2accel[3]);
		}

		// combine them together
		target_accel[axis] = ff[axis];
// 		target_accel[axis] = 0;
		for(int i=0; i<3; i++)
			target_accel[axis] += pid[axis][i] * rate2accel[i];		// TODO: factor
	}

	// TODO: handle accel limitation

	return 0;
}


int pos_controller::accel_to_lean_angles(float dt)
{
	// rotate from north-east to forward-right axis
	float accel_forward = cos_yaw * target_accel[0] + sin_yaw * target_accel[1];
	float accel_right = -sin_yaw * target_accel[0] + cos_yaw * target_accel[1];
	
	// accel to lean angle
	float new_euler_pitch  = atan2(-accel_forward, G_in_ms2);
	float new_euler_roll = atan2(accel_right*cos(eulers[1]), G_in_ms2);

	if (fabs(new_euler_roll) > quadcopter_range[0] || fabs(new_euler_pitch) > quadcopter_range[1])
	{
		limit_angle = true;
		// TODO: handle angle limitation correctly
		float factor = fmin(quadcopter_range[0] / fabs(new_euler_roll), quadcopter_range[1] / fabs(new_euler_pitch));
		new_euler_roll *= factor;
		new_euler_pitch *= factor;
	}
	else
	{
		limit_angle = false;
	}


	// max rotation speed: 100 degree/s, then apply a 5hz LPF
	float delta_roll = new_euler_roll - target_euler[0];
	float delta_pitch = new_euler_pitch - target_euler[1];

	float max_delta = dt * 100 * PI / 180;
// 	delta_roll = limit(delta_roll, -max_delta, max_delta);
// 	delta_pitch = limit(delta_pitch, -max_delta, max_delta);

	float alpha5 = dt / (dt + 1.0f/(2*PI * 12.0f));

	target_euler[0] = (target_euler[0] + delta_roll) * alpha5 + (1-alpha5) * target_euler[0];
	target_euler[1] = (target_euler[1] + delta_pitch) * alpha5 + (1-alpha5) * target_euler[1];

	return 0;
}
