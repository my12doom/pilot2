#include "mode_poshold.h"
#include "pilot.h"
#include <utils/param.h>

static param use_EKF("ekf", 0);		// use EKF estimator


int flight_mode_poshold::setup()
{
	if (!yap.pos_estimator_ready())
	{
		LOGE("poshold failed: position estimator not ready\n");
		return -1;
	}

	yap.alt_controller.reset();

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

	return 0;
}

int flight_mode_poshold::exit()
{
	return 0;
}
int flight_mode_poshold::loop(float dt)
{
	// altitude
	yap.default_alt_controlling();

	// attitude, roll and pitch
	// airborne or armed and throttle up
	bool after_unlock_action = yap.airborne || yap.rc[2] > 0.55f;
	if (after_unlock_action)	// airborne or armed and throttle up
	{
		// 10hz pos controller rate
		static int64_t last_pos_controll_time = 0;
		float pos_control_dt = (systimer->gettime() - last_pos_controll_time) / 1000000.0f;
		if (pos_control_dt > 0.1f)
		{
			last_pos_controll_time = systimer->gettime();
			if (pos_control_dt < 1.0f)
			{
				float ne_pos[2];
				float ne_velocity[2];
				yap.get_pos_velocity_ned(ne_pos, ne_velocity);
				float desired_velocity[2] = {yap.rc[1] * 5, yap.rc[0] * 5};
				if (abs(desired_velocity[0]) < 0.4f)
					desired_velocity[0] = 0;
				if (abs(desired_velocity[1]) < 0.4f)
					desired_velocity[1] = 0;

				float euler_target[3] = {NAN, NAN, NAN};
				yap.pos_control.provide_attitue_position(euler, ne_pos, ne_velocity);
				yap.pos_control.set_desired_velocity(desired_velocity);
				yap.pos_control.update_controller(pos_control_dt);
				yap.pos_control.get_target_angles(euler_target);

				euler_target[2] = NAN;
				yap.attitude_controll.set_euler_target(euler_target);
			}
		}

		// yaw
		float stick[3] = {NAN, NAN, yap.rc[3]};
		yap.attitude_controll.update_target_from_stick(stick, dt);

	}
	else
	{
		float euler_target[3] = {0,0, euler[2]};
		yap.attitude_controll.set_euler_target(euler_target);
	}

	return 0;
}
