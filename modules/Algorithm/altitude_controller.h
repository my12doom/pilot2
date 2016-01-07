#pragma once

#include <stdint.h>
#include <math.h>

class altitude_controller
{
public:
	altitude_controller();
	~altitude_controller();

	// provide system state estimation for controller
	// alt[0-2] : altitude, climb_rate, acceleration
	// sonar: sonar reading ,NAN for invalid reading(no echo or other errors), controller is responsible for filtering and switching between sonar and baro
	//		for advanced altitude estimator with sonar fusing: pass sonar as NAN.
	// attitude[0-3], euler angle [roll,pitch,yaw]
	// throttle_realized: throttle delivered last tick by motor mixer.
	// motor_state: a combination of one or more of THROTTLE_LIMIT_MIN, THROTTLE_LIMIT_MAX.
	//		MOTOR_LIMIT_NONE : the total power of motor matrix didn't reached any limitation 
	//		THROTTLE_LIMIT_MAX : the total power of motor matrix has reached maximum 
	//		THROTTLE_LIMIT_MAX : the total power of motor matrix has reached minimum 
	// airborne: if the aircraft has liftoff
	int provide_states(float *alt, float sonar, float *attitude, float throttle_realized, int motor_state, bool airborne);

	// set the desired altitude target directly.
	// (not yet implemented)
	int set_altitude_target(float new_target);
	
	// update the controller
	// dt: time interval
	// user_rate: user desired climb rate, usually from stick.
	int update(float dt, float user_rate);

	// reset controller
	// call this if the controller has just been engaged
	// the controller will try to output low throttle if not airborne,
	// or maintain current altitude.
	int reset();

	//get altitude target 
	float get_altitude_state();

	// return whether this controller is used( update() called in less than 0.1 seconds)
	bool used();
	
	float get_result();

	// return true if sonar surface tracking enabled
	bool sonar_actived();

	// estimated hover throttle
	float throttle_hover;

	float m_baro_states[3];
	float m_attitude[3];
	float m_throttle_realized;
	int m_motor_state;
	bool m_airborne;

	int64_t last_update;
	float baro_target;// = 0;
	float target_climb_rate;// = 0;
	float target_accel;// = 0;
	float altitude_error_pid[3];// = {0};
	float climb_rate_error_pid[3];// = {0};
	float accel_error_pid[3];// = {0};
	float throttle_result;// = 0;

	float m_sonar;
	float m_last_valid_sonar;
	float m_sonar_target;
	float m_sonar_ticker;

protected:
	float feed_forward_factor;
	float calc_leash_length(float speed, float accel, float kP);
};
