#include "pos_controll.h"
#include <string.h>
#include <math.h>
#include <float.h>
#include <Protocol/common.h>
#include <utils/param.h>

// constants
static float leash_pos = 6.0f;
static float leash = 5.0f;
static float linear_distance = 5.0f;
static float max_speed = 5.0f;
static float max_speed_ff = 2.0f;
static bool use_desired_feed_forward = false;
static float feed_forward_factor = 1;
static float rate2accel[4] = {1.5f, 0.5f, 0.2f, 2.0f};
static float pos2rate_P = 1.0f;


// "parameters/constants"
static param quadcopter_range[3] = 
{
	param("rngR", PI / 5),			// roll
	param("rngP", PI / 5),			// pitch
	param("rngY", PI / 8),			// yaw
};

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

static float sqrt2(float in)
{
	float o = in>0?1:-1;
	return o*sqrt(fabs(in));
}

pos_controller::pos_controller()
{
	release_stick_timer = 0;
#ifdef WIN32
	f = fopen("Z:\\log.csv", "wb");
	fprintf(f, "time,v,tv,p,sp\r\n");
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

	state = braking;

	return 0;
}

int pos_controller::set_desired_velocity(float *desired_velocity)
{
	memcpy(this->desired_velocity, desired_velocity, sizeof(this->desired_velocity));

	// rotate from body frame to earth frame
	desired_velocity_earth[0] = cos_yaw * desired_velocity[0] - sin_yaw * desired_velocity[1];
	desired_velocity_earth[1] = sin_yaw * desired_velocity[0] + cos_yaw * desired_velocity[1];

	pilot_angle[0] = NAN;
	pilot_angle[1] = NAN;

	return 0;
}

int pos_controller::set_desired_stick(float *stick)
{
	float desired_velocity[2] = {stick[1] * 5, stick[0] * 5};
	if (abs(desired_velocity[0]) < 0.4f)
		desired_velocity[0] = 0;
	if (abs(desired_velocity[1]) < 0.4f)
		desired_velocity[1] = 0;

	set_desired_velocity(desired_velocity);

	pilot_angle[0] = stick[0] * quadcopter_range[0];
	pilot_angle[1] = -stick[1] * quadcopter_range[0];	// pitch stick and coordinate are reversed

	return 0;
}

int pos_controller::update_controller(float dt)
{
	// if user released stick, or stick moving toward set point.
	// apply a extra braking until velocity decreased to a acceptable value, for at most 1 second.
	bool stick_released = fabs(desired_velocity_earth[0]) < 0.01f && fabs(desired_velocity_earth[1]) < 0.01f;
	if (stick_released)
		release_stick_timer += dt;
	else
		release_stick_timer = 0;

	min_braking_speed = 1.0f * 1.0f;
	float delta[2] = {setpoint[0] - pos[0], setpoint[1] - pos[1]};

	if  (	(velocity[0]*velocity[0] + velocity[1]*velocity[1] > min_braking_speed) &&					// speed fast enough(1m/s)?
			(
			  (stick_released && release_stick_timer < 1.0f)											// released stick? less than 1 second?
			  || (delta[0]*desired_velocity_earth[0] + delta[1]*desired_velocity_earth[1] < 0)			// moving toward set point?
			)
		)
	{
		// braking is based on a 0.05hz low pass filter that move set point toward current position
		float alpha = dt / (dt + 1.0f/(2*PI * 0.05f));
		setpoint[0] = alpha * pos[0] + (1-alpha) * setpoint[0];
		setpoint[1] = alpha * pos[1] + (1-alpha) * setpoint[1];
	}

	move_desire_pos(dt);

	pos_to_rate(dt);

	rate_to_accel(dt);

	accel_to_lean_angles();

#ifdef WIN32
	fprintf(f, "%.3f,%f,%f,%f,%f\r\n", (GetTickCount()-tick)/1000.0f, velocity[0],target_velocity[0], pid[0][0], pid[0][1]);
	fflush(f);
#endif

	return 0;
}

int pos_controller::get_target_angles(float *target_angles)
{
	memcpy(target_angles, this->target_euler, sizeof(this->target_euler));

	return 0;
}

int pos_controller::set_setpoint(float *pos)
{
	// reset pid, reset desired pos
	memset(pid, 0, sizeof(pid));
	pid[0][2] = NAN;
	pid[1][2] = NAN;
	last_target_velocity[0] = NAN;
	ff[0] = 0;
	ff[1] = 0;

	setpoint[0] = pos[0];
	setpoint[1] = pos[1];

	// lean angle to accel
	float accel_target[2];		// [forward, right]
	accel_target[0] = G_in_ms2 * (-sin(eulers[1])/cos(eulers[1]));		// lean forward = negetive pitch angle
	accel_target[1] = G_in_ms2 * (sin(eulers[0])/cos(eulers[0]));

	// rotate accel from forward-right to north-east axis
	float accel_north = cos_yaw * accel_target[0] - sin_yaw * accel_target[1];
	float accel_east = sin_yaw * accel_target[0] + cos_yaw * accel_target[1];

	// set integrator to correct numbers to achieve smooth transition
	if (rate2accel[1] > 0)
	{
		pid[0][1] = accel_north / rate2accel[1];
		pid[1][1] = accel_east / rate2accel[1];
	}
	else
	{
		pid[0][1] = 0;
		pid[1][1] = 0;
	}

	// TODO : do other checking and initializing

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

	if ( new_distance < leash_pos || new_distance < old_distance)
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
	if (!isnan(last_target_velocity[0]))
	{
		float alpha = dt / (dt + 1.0f/(2*PI * 5.5f));

		float ff0 = (target_velocity[0] - last_target_velocity[0]) / dt;
		float ff1 = (target_velocity[1] - last_target_velocity[1]) / dt;

		ff[0] = ff[0] * (1-alpha) + ff0 * alpha;
		ff[1] = ff[1] * (1-alpha) + ff1 * alpha;
	}
	last_target_velocity[0] = target_velocity[0];
	last_target_velocity[1] = target_velocity[1];

	// 30hz LPF for D term
	float alpha30 = dt / (dt + 1.0f/(2*PI * 5.0f));

	for(int axis=0; axis<2; axis++)
	{
		float p = target_velocity[axis] - velocity[axis];
		float d = (p - pid[axis][0]) / dt;

		pid[axis][2] = isnan(pid[axis][2]) ? 0 : (pid[0][2] * (1-alpha30) + d * alpha30);
		pid[axis][0] = p;

		// update I only if we did not hit accel/angle limit or throttle limit or I term will reduce
		if (true)
		{
			pid[axis][1] += pow((double)fabs(p), 0.3) * (p>0?1:-1) * dt;
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


int pos_controller::accel_to_lean_angles()
{
	// rotate from north-east to forward-right axis
	float accel_forward = cos_yaw * target_accel[0] + sin_yaw * target_accel[1];
	float accel_right = -sin_yaw * target_accel[0] + cos_yaw * target_accel[1];

	// accel to lean angle
	target_euler[1] = atan2(-accel_forward, G_in_ms2);
	target_euler[0] = atan2(accel_right/**cos(eulers[1])*/, G_in_ms2);		// maybe target_pitch not needed?

	// TODO: handle angle limitation correctly
	target_euler[0] = limit(target_euler[0], -PI/5, PI/5);
	target_euler[1] = limit(target_euler[1], -PI/5, PI/5);

	return 0;
}
