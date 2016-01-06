#include "mode_RTL.h"
#include "pilot.h"
#include <utils/param.h>
#include <utils/log.h>


static param RTL_turning_speed("rtlT", PI/4);		// RTL yaw turning speed, radian/s
static param RTL_max_climbing_speed("rtlC", 3.0);	// RTL climbing speed
static param RTL_altitude("rtlA", 15.0);			// RTL altitude
static param RTL_decending_speed("rtlD", 2.0);		// RTL descending speed ( not yet implemented)
static param RTL_moving_speed("rtlV", 5.0);			// RTL moving speed( not yet implemented)
static param RTL_loiter_time("rltL", 10);			// RTL loiter at home time

static param pid_factor_yaw("sP3", 8);
static param pid_factor_alt("altP", 1);

int flight_mode_RTL::setup()
{
	yap.alt_controller.reset();

	// is position estimator ready?
	if (!yap.pos_estimator_ready())
	{
		LOGE("RTL failed: position not ready\n");
		return -1;
	}

	// reset pos controller
	position_meter meter = yap.estimator.get_estimation_meter();
	float ne_pos[2] = {meter.latitude, meter.longtitude};
	float ne_velocity[2] = {meter.vlatitude, meter.vlongtitude};
	float desired_velocity[2] = {0, 0};
	float euler_target[3];

	yap.pos_control.provide_attitue_position(euler, ne_pos, ne_velocity);
	yap.pos_control.set_desired_velocity(desired_velocity);
	yap.pos_control.get_target_angles(euler_target);
	yap.pos_control.reset();
	euler_target[2] = NAN;
	yap.attitude_controll.set_euler_target(euler_target);

	// reset all tick
	turn_around_tick = 0;
	loiter_tick = 0;
	rising_tick = 0;
	move_tick = 0;

	// record current position and home
	yap.get_pos_velocity_ned(start_pos_ne, NULL);
	home_pos_ne[0] = yap.home[0];
	home_pos_ne[1] = yap.home[1];

	float bearing_north = home_pos_ne[0] - ne_pos[0];
	float bearing_east = home_pos_ne[1] - ne_pos[1];
	float distance = sqrt(bearing_north * bearing_north + bearing_east * bearing_east);

	if (distance > 5.0f)
	{
		LOGE("RTL distance: %.2f, turning around\n", distance);
		stage = turn_around;
	}
	else
	{
		LOGE("RTL distance: %.2f, move toward home directly\n", distance);
		stage = move;
	}

	if (yap.flashlight)
		yap.flashlight->on();

	LOGE("RTL start, pos=%.2f,%.2f, home=%.2f,%.2f\n", start_pos_ne[0], start_pos_ne[1], home_pos_ne[0], home_pos_ne[1]);

	return 0;
}

int flight_mode_RTL::exit()
{
	return 0;
}

int flight_mode_RTL::loop(float dt)
{
	// make position controller happy
	float ne_pos[2];
	float ne_velocity[2];
	float desired_velocity[2] = {0,0};
	yap.get_pos_velocity_ned(ne_pos, ne_velocity);
	yap.pos_control.provide_attitue_position(euler, ne_pos, ne_velocity);
	yap.pos_control.set_desired_velocity(desired_velocity);

	// bearing
	float bearing_north = home_pos_ne[0] - ne_pos[0];
	float bearing_east = home_pos_ne[1] - ne_pos[1];
	float bearing = atan2(bearing_east, bearing_north);
	float distance = sqrt(bearing_north * bearing_north + bearing_east * bearing_east);
	if (distance < 5)
		bearing = yap.attitude_controll.euler_sp[2];

	switch (stage)
	{
	case turn_around:
		{
			// hold position and altitude
			yap.alt_controller.update(dt, 0);
			yap.pos_control.set_setpoint(start_pos_ne, false);

			// spin slowly ( default 45 degree/s) toward bearing
			float yaw_delta = radian_sub(bearing, euler[2]);
			yaw_delta = limit(yaw_delta, -RTL_turning_speed/pid_factor_yaw, RTL_turning_speed/pid_factor_yaw);
			float yaw_target = radian_add(euler[2], yaw_delta);
			float euler_sp[3] = {NAN, NAN, yaw_target};
			yap.attitude_controll.set_euler_target(euler_sp);

			// have we reached desired yaw?
			if (fabs(radian_sub(euler[2], bearing)) < 10 * PI / 180)
			{
				stage = rise;
				turn_around_tick += dt;

				if (turn_around_tick > 1)
				{
					stage = rise;

					LOGE("RTL: turn around done\n");
				}
			}
			else
			{
				turn_around_tick = 0;
			}
		}
		break;


	case rise:
		{
			// hold position and yaw
			yap.pos_control.set_setpoint(start_pos_ne, false);
			float euler_sp[3] = {NAN, NAN, bearing};
			yap.attitude_controll.set_euler_target(euler_sp);

			// rise up
			float speed = (yap.takeoff_ground_altitude + RTL_altitude - yap.alt_estimator.state[0]) * pid_factor_alt;
			speed = limit(speed, 0, RTL_max_climbing_speed);
			yap.alt_controller.update(dt, speed);

			// have we reached desired altitude?
			if (yap.alt_estimator.state[0] > yap.takeoff_ground_altitude + RTL_altitude - 1)
			{
				rising_tick += dt;
				if (rising_tick > 1)
				{
					stage = move;
					LOGE("RTL: rising done, alt=%.2f\n", yap.alt_estimator.state[0]);
				}
			}
			else
			{
				rising_tick = 0;
			}
		}
		break;


	case move:
	case loiter:
		{
			// hold altitude and yaw
			yap.alt_controller.update(dt, 0);
// 			float euler_sp[3] = {NAN, NAN, bearing};
// 			yap.attitude_controll.set_euler_target(euler_sp);

			// move toward home
			yap.pos_control.set_setpoint(home_pos_ne, false);

			// have we reached home?
			if (stage == move)
			{
				if (distance < 2.0f)
				{
					move_tick += dt;
					if (move_tick > 1.0f)
					{
						LOGE("RTL: moving done, pos=%.2f,%.2f\n", ne_pos[0], ne_pos[1]);
						stage = loiter;
					}
				}
				else
				{
					move_tick = 0;
				}
			}
			else if (stage == loiter)
			{
				loiter_tick += dt;

				if (loiter_tick > RTL_loiter_time)
				{
					LOGE("RTL: loiter done, pos=%.2f,%.2f\n", ne_pos[0], ne_pos[1]);
					stage = down;
				}
			}

		}
		break;

	case down:
		{
			// hold position
			yap.pos_control.set_setpoint(home_pos_ne, false);

			// go down, using default alt controlling and islanding=true
			yap.islanding = true;
			yap.default_alt_controlling();
		}
		break;
	}

	// position
	float euler_target[3] = {NAN, NAN, NAN};
	yap.pos_control.update_controller(dt);
	yap.pos_control.get_target_angles(euler_target);
	euler_target[2] = NAN;

	// attitude
	yap.attitude_controll.set_euler_target(euler_target);

	// throttle;
	yap.throttle_result = yap.alt_controller.get_result();

	if (!yap.pos_estimator_ready())
		yap.new_event(event_pos_bad, 0);

	// log
	s.euler_setpoint[0] = yap.attitude_controll.euler_sp[0];
	s.euler_setpoint[1] = yap.attitude_controll.euler_sp[1];
	s.euler_setpoint[2] = yap.attitude_controll.euler_sp[2];

	s.pos[0] = ne_pos[0];
	s.pos[1] = ne_pos[1];

	s.pos_setpoint[0] = yap.pos_control.setpoint[0];
	s.pos_setpoint[1] = yap.pos_control.setpoint[1];
	
	s.baro_setpoint = yap.alt_controller.baro_target;
	s.sonar_setpoint = yap.alt_controller.m_sonar_target;

	log2(&s, TAG_RTL_STATE, sizeof(s));

	return 0;
}
