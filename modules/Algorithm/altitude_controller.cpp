#include "altitude_controller.h"
#include <string.h>
#include <Protocol/common.h>
#include <utils/param.h>
#include <HAL/Interface/Interfaces.h>

param default_throttle_hover("hov", 0.35f);
#define sonar_step_threshold 3.0f

static param quadcopter_max_climb_rate("maxC",5);
static param quadcopter_max_descend_rate("maxD", 2);
static param quadcopter_max_acceleration("maxA", 12.5f);
static param alt_braking_enabled("hbra", 1);
static param pid_quad_altitude[4] = 	// P, I, D, IMAX, 
										// unit: 1/second, 1/seconds^2, 1, meter*second
										// convert altitude error(meter) to target climb rate(meter/second)
{
	param("altP", 1.0f),
	param("altI", 0.0f),
	param("altD", 0.0f),
	param("altM", 0.0f),
};

static param pid_quad_alt_rate[4] = 	// P, I, D, IMAX
										// unit: 1/second, 1/seconds^2, 1, meter*second
										// convert climb rate error(meter/second) to target acceleration(meter/second^2)
{
	param("cliP", 5.0f),
	param("cliI", 0.0f),
	param("cliD", 0.0f),
	param("cliM", 0.0f),
};
static param pid_quad_accel[4] =		// P, I, D, IMAX
										// unit:
										// convert acceleration error(meter/second^2) to motor output
										// In ardupilot, default P = 0.75 converts 1 cm/s^2 into 0.75 * 0.1% of full throttle
										// In yetanotherpilot implementation, default P=0.075 converts 1 m/s^2 into 0.075 of full throttle
										// the max accel error in default value is around +- 6.66 m/s^2, but should not use that much
{
	param("accP", 0.075f),
	param("accI", 0.150f),
	param("accD", 0.0f),
	param("accM", 2.5f),
};

altitude_controller::altitude_controller()
:throttle_hover(0)
,baro_target(0)
,target_climb_rate(0)
,target_accel(0)
,throttle_result(0)
,m_sonar_ticker(0)
{
	memset(altitude_error_pid, 0, sizeof(altitude_error_pid));
	memset(climb_rate_error_pid, 0, sizeof(climb_rate_error_pid));
	memset(climb_rate_error_pid, 0, sizeof(climb_rate_error_pid));
	last_update = 0;
	climb_rate_override = NAN;
}

altitude_controller::~altitude_controller()
{

}

static inline float fmax(float a, float b)
{
	return a>b?a:b;
}

// provide system state estimation for controller
// alt[0-2] : altitude, climb_rate, acceleration
// sonar: sonar reading ,NAN for invalid reading(no echo or other errors), controller is responsible for filtering and switching between sonar and baro
// throttle_realized: throttle delivered last tick by motor mixer.
// motor_state: a combination of one or more of MOTOR_LIMIT_MIN, MOTOR_LIMIT_MAX.
//		MOTOR_LIMIT_NONE : the total power of motor matrix didn't reached any limitation 
//		MOTOR_LIMIT_MAX : the total power of motor matrix has reached maximum 
//		MOTOR_LIMIT_MAX : the total power of motor matrix has reached minimum 
// airborne: if the aircraft has liftoff
int altitude_controller::provide_states(float *alt, float sonar, float *attitude, float throttle_realized, int motor_state, bool airborne)
{
	m_baro_states[0] = alt[0];
	m_baro_states[1] = alt[1];
	m_baro_states[2] = alt[2];
	m_attitude[0] = attitude[0];
	m_attitude[1] = attitude[1];
	m_attitude[2] = attitude[2];
	m_throttle_realized = throttle_realized;
// 	m_motor_state = motor_state;
	m_airborne = airborne;
	m_sonar = sonar;
	
	if (!isnan(sonar))
	{
		// step response
		// 0.1f : sonar update interval
		float sonar_update_inteval = 0.1f;
		float step_change_max = sonar_step_threshold * (fabs(m_baro_states[1]) * sonar_update_inteval + 0.5f * fabs(m_baro_states[2]) * sonar_update_inteval * sonar_update_inteval);
		step_change_max = fmax(step_change_max, 0.2f);
		if (isnan(climb_rate_override) && !isnan(m_sonar_target) &&  fabs(sonar - m_last_valid_sonar) >  step_change_max) // note: only do step response with no climb rate override
		{
			m_sonar_target += sonar - m_last_valid_sonar;
			start_braking();
		}

		m_last_valid_sonar = sonar;
	}

	return 0;
}

// set the desired altitude target directly.
// (not yet implemented)
int altitude_controller::set_altitude_target(float new_target)
{
	float &alt_target = isnan(m_sonar_target) ?(baro_target) : m_sonar_target;
	alt_target=new_target;
	return -1;
}
float altitude_controller::get_altitude_state()
{
	float &alt_state = isnan(m_sonar_target) ? m_baro_states[0]: m_last_valid_sonar;
	return alt_state;
}

// update the controller
// dt: time dt
// user_rate: user desired climb rate, usually from stick.
int altitude_controller::update(float dt, float user_rate)
{
	if (systimer->gettime() - last_update > 1000000)
		reset();

	if (throttle_hover <= 0.01f || isnan(throttle_hover))
		throttle_hover = default_throttle_hover;

	// sonar switching	
	if (isnan(m_sonar) == isnan(m_sonar_target))				// reset ticker if sonar state didn't changed
		m_sonar_ticker = 0;

	if (m_sonar_ticker < 0.3f)
	{
		m_sonar_ticker += dt;

		if (m_sonar_ticker > 0.3f)
		{
			// sonar state changed more than 0.5 second.
			if (isnan(m_sonar))
			{
				baro_target = m_baro_states[0] + m_sonar_target - m_last_valid_sonar;
				m_sonar_target = NAN;
				LOGE("changed to baro: %f/%f,%f,%f\n", m_sonar_target, m_sonar, baro_target, m_baro_states[0]);
			}
			else
			{
				m_sonar_target = m_sonar + baro_target - m_baro_states[0];
				LOGE("changed to sonar: %f/%f,%f,%f\n", m_sonar_target, m_sonar, baro_target, m_baro_states[0]);
			}
		}
	}

	float leash_up = calc_leash_length(quadcopter_max_climb_rate, quadcopter_max_acceleration, pid_quad_altitude[0]);
	float leash_down = calc_leash_length(quadcopter_max_descend_rate, quadcopter_max_acceleration, pid_quad_altitude[0]);
	float &alt_target = isnan(m_sonar_target) ? baro_target : m_sonar_target;
	float &alt_state = isnan(m_sonar_target) ? m_baro_states[0]: m_last_valid_sonar;
		
	// only move altitude target if throttle and target climb rate didn't hit limits, and no climb target override exists
	if (isnan(climb_rate_override) && 
		(
		(!(m_motor_state & LIMIT_POSITIVE_HARD) && user_rate > 0 && (target_climb_rate < quadcopter_max_climb_rate)) || 
		(!(m_motor_state & LIMIT_NEGATIVE_HARD) && user_rate < 0 && (target_climb_rate > -quadcopter_max_descend_rate))
		)
		)
	{
		alt_target += user_rate * dt;
	}

	alt_target = limit(alt_target, alt_state-leash_down, alt_state+leash_up);

	// new target rate, directly use linear approach since we use very tight limit 
	// TODO: use sqrt approach on large errors (see get_throttle_althold() in Attitude.pde)
	altitude_error_pid[0] = alt_target - alt_state;
	altitude_error_pid[0] = limit(altitude_error_pid[0], -leash_down, leash_up);
	target_climb_rate = pid_quad_altitude[0] * altitude_error_pid[0];
	if (!isnan(climb_rate_override))
		target_climb_rate = climb_rate_override;

	// feed forward
	// 
	feed_forward_factor += m_airborne ? -dt : dt;
	feed_forward_factor = limit(feed_forward_factor, 0.35f, 0.8f);
	target_climb_rate += user_rate * feed_forward_factor;

	TRACE("\rtarget_climb_rate=%.2f climb from alt =%.2f, user=%.2f, out=%2f.     ", target_climb_rate, target_climb_rate-user_rate * feed_forward_factor, user_rate, throttle_result);

	// a better outter loop for centered throttle stick remote controller.
	if (int(alt_braking_enabled) && m_airborne)
	{
		bool near_ground = !isnan(m_sonar_target) && m_last_valid_sonar < 0.15f;
		static bool last_near_ground;
		if (!last_near_ground && near_ground)
			m_sonar_target = m_last_valid_sonar + m_baro_states[1] / 2;
		last_near_ground = near_ground;


		if (!near_ground)
		{
			bool stick_centered = fabs(user_rate) < 0.1f;

			if (!stick_centered)
			{
				m_controller_state = alt_override;
				m_braking_timer = 0;
				target_climb_rate = user_rate;
			}
			else
			{
				if (m_controller_state == alt_override)
				{
					m_controller_state = alt_braking;
					m_braking_timer = 0;
					target_climb_rate = 0;
				}

				else if (m_controller_state == alt_braking)
				{
					if (fabs(m_baro_states[1]) < 0.5f)
						m_braking_timer += dt;
					else
						m_braking_timer = 0;

					if (m_braking_timer > 0.5f)
					{
						float &target = isnan(m_sonar_target) ? baro_target : m_sonar_target;
						target = (isnan(m_sonar_target) ? m_baro_states[0] : m_last_valid_sonar) + m_baro_states[1] /2;
						LOGE("alt braking done, target set to %.2f(%s)", target, isnan(m_sonar_target) ? "baro" : "sonar");
						m_controller_state = alt_hold;
					}
					target_climb_rate = 0;
				}
				else if (m_controller_state == alt_hold)
				{
					// do nothing
				}
			}
		}
	}

	target_climb_rate = limit(target_climb_rate, -quadcopter_max_descend_rate, quadcopter_max_climb_rate);

	// new climb rate error
	float climb_rate_error = target_climb_rate - m_baro_states[1];
	//climb_rate_error = limit(climb_rate_error, -quadcopter_max_descend_rate, quadcopter_max_climb_rate);

	// apply a 2Hz LPF to rate error
	const float RC2 = 1.0f/(2*3.1415926 * 3.5f);
	float alpha2 = dt / (dt + RC2);
	// 5Hz LPF filter
	const float RC5 = 1.0f/(2*3.1415926 * 5.0f);
	float alpha5 = dt / (dt + RC5);
	// 30Hz LPF for derivative factor
	const float RC30 = 1.0f/(2*3.1415926 * 30.0f);
	float alpha30 = dt / (dt + RC30);

	// TODO: add feed forward
	// reference: get_throttle_rate()

	climb_rate_error_pid[0] = isnan(climb_rate_error_pid[0]) ? (climb_rate_error) : (climb_rate_error_pid[0] * (1-alpha2) + alpha2 * climb_rate_error);
	target_accel = climb_rate_error_pid[0] * pid_quad_alt_rate[0];
	target_accel = limit(target_accel,  m_airborne ? -quadcopter_max_acceleration : -2 * quadcopter_max_acceleration, quadcopter_max_acceleration);


	// new accel error, +2Hz LPF
	float accel_error = target_accel - m_baro_states[2];
	if (isnan(accel_error_pid[0]))
	{
		accel_error_pid[0] = accel_error;
	}
	else
	{
		accel_error = accel_error_pid[0] * (1-alpha2) + alpha2 * accel_error;
	}

	// core pid
	// only integrate if throttle didn't hit limits or I term will reduce
	if (m_airborne)
	if ((!(m_motor_state & LIMIT_POSITIVE_HARD) && accel_error_pid[0] > 0) || (!(m_motor_state & LIMIT_NEGATIVE_HARD) && accel_error_pid[0] < 0))
	{
		accel_error_pid[1] += accel_error_pid[0] * dt;
		accel_error_pid[1] = limit(accel_error_pid[1], -pid_quad_accel[3], pid_quad_accel[3]);
	}
	float D = (accel_error - accel_error_pid[0])/dt;
	accel_error_pid[2] = isnan(accel_error_pid[2])? D : (accel_error_pid[2] * (1-alpha30) + alpha30 * D);
	accel_error_pid[0] = accel_error;


	float output = 0;
	output += accel_error_pid[0] * pid_quad_accel[0];
	output += accel_error_pid[1] * pid_quad_accel[1];
	output += accel_error_pid[2] * pid_quad_accel[2];
	output *= throttle_hover / default_throttle_hover;			// normalize throttle output PID, from throttle percentage to acceleration

	throttle_result  = output + throttle_hover;
	float angle_boost_factor = 1/ cos(m_attitude[0]) / cos(m_attitude[1]);
	if (angle_boost_factor < 0)
		angle_boost_factor = 0;		// if we got reverted, kill thrust
	else if (angle_boost_factor > 1.5f)
		angle_boost_factor = 1.5f;
	else if (angle_boost_factor < 1.0f)
		angle_boost_factor = 1.0f;

	throttle_result = throttle_result * angle_boost_factor;

	if (throttle_result > 1 - QUADCOPTER_THROTTLE_RESERVE)
	{
		m_motor_state = LIMIT_POSITIVE_HARD;
	}
	else if (throttle_result < 0)
	{
		m_motor_state = LIMIT_NEGATIVE_HARD;
	}
	else
	{
		m_motor_state = 0;
	}
	throttle_result = limit(throttle_result, 0.0f, 1.0f);

	TRACE("\rthrottle=%f, altitude = %.2f/%.2f, pid=%.2f,%.2f,%.2f, limit=%d", throttle_result, m_baro_states[0], baro_target,
		accel_error_pid[0], accel_error_pid[1], accel_error_pid[2], m_motor_state);

	// update throttle_real_crusing if we're in near level state and no violent climbing/descending action
	if (m_airborne && m_throttle_realized>0 && fabs(m_baro_states[1]) < 0.5f && fabs(m_baro_states[2])<0.5f && fabs(m_attitude[0])<5*PI/180 && fabs(m_attitude[1])<5*PI/180
		&& fabs(user_rate) < 0.001f)
	{
		// 0.2Hz low pass filter
		const float RC02 = 1.0f/(2*3.1415926 * 0.02f);
		float alpha02 = dt / (dt + RC02);

		throttle_hover = throttle_hover * (1-alpha02) + alpha02 * m_throttle_realized;
		// TODO: estimate throttle cursing correctly

		TRACE("\rthrottle_hover=%f", throttle_hover);
	}

	last_update = systimer->gettime();

	return 0;
}

int altitude_controller::start_braking()
{
	if (!int(alt_braking_enabled))
		return -1;

	m_controller_state = alt_override;
	m_braking_timer = 0;

	return 0;
}

bool altitude_controller::used()
{
	if (systimer->gettime() - last_update < 100000)
		return true;
	return false;
}

// reset controller
// call this if the controller has just been engaged
// the controller will try to output low throttle if not airborne,
// or maintain current altitude.
int altitude_controller::reset()
{
	if (used())
		return 0;

	baro_target = m_airborne ? m_baro_states[0] : (m_baro_states[0]-1.2f);
	feed_forward_factor = m_airborne ? 0.35f : 0.35f;
	accel_error_pid[0] = NAN;
	accel_error_pid[1] = 0;
	accel_error_pid[2] = NAN;
	climb_rate_error_pid[0] = NAN;
	m_sonar_ticker = 0;
	m_sonar_target = NAN;
	m_controller_state = alt_hold;

	LOGE("altitude_controller::reset()\n");

	return 0;
}

float altitude_controller::get_result()
{
	return throttle_result;
}

bool altitude_controller::sonar_actived()
{
	return !isnan(m_sonar_target);
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float altitude_controller::calc_leash_length(float speed, float accel, float kP)
{
	float leash_length;

	// sanity check acceleration and avoid divide by zero
	if (accel <= 0.0f) {
		accel = 5.0f;
	}

	// avoid divide by zero
	if (kP <= 0.0f) {
		return 1;
	}

	// calculate leash length
	if(speed <= accel / kP) {
		// linear leash length based on speed close in
		leash_length = speed / kP;
	}else{
		// leash length grows at sqrt of speed further out
		leash_length = (accel / (2.0f*kP*kP)) + (speed*speed / (2.0f*accel));
	}

	// ensure leash is at least 1m long
	if( leash_length < 1 ) {
		leash_length = 1;
	}

	return leash_length;
}