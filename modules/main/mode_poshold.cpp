#include "mode_poshold.h"
#include "pilot.h"
#include <utils/param.h>

static param use_EKF("ekf", 0);		// use EKF estimator


bool flight_mode_poshold::is_ready()
{
	if (!yap.get_estimator_state())
	{
		LOGE("poshold failed: position estimator not ready\n");
		return false;
	}

	return true;
}

int flight_mode_poshold::setup()
{
	yap.alt_controller.reset();

	// reset pos controller
	position_meter meter = yap.estimator.get_estimation_meter();
	float ne_pos[2] = {meter.latitude, meter.longtitude};
	float ne_velocity[2] = {meter.vlatitude, meter.vlongtitude};
	float desired_velocity[2] = {0, 0};
	float euler_target[3];

	yap.pos_control->provide_attitue_position(euler, ne_pos, ne_velocity);
	yap.pos_control->set_desired_velocity(desired_velocity);
	yap.pos_control->get_target_angles(euler_target);
	yap.pos_control->reset();
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
	if (yap.pos_control_hybird.state == pos_controller::braking && yap.get_estimator_state() == fully_ready)
		yap.alt_controller.start_braking();

	// attitude, roll and pitch
	// airborne or armed and throttle up
	bool after_unlock_action = yap.airborne || yap.rc[2] > 0.55f;
	if (after_unlock_action)	// airborne or armed and throttle up
	{
		if (dt < 1.0f)
		{
			// hacks
			if (yap.get_estimator_state() == transiting)
			{
				yap.pos_control_hybird.set_state_machine(pos_controller::braking);
				yap.pos_control_hybird.low_speed_tick = 0;
			}

			float ne_pos[2];
			float ne_velocity[2];
			yap.get_pos_velocity_ned(ne_pos, ne_velocity);

			float euler_target[3] = {NAN, NAN, NAN};
			yap.pos_control->provide_attitue_position(euler, ne_pos, ne_velocity);
			yap.pos_control->set_desired_stick(yap.rc);
			yap.pos_control->update_controller(dt);
			yap.pos_control->get_target_angles(euler_target);

			euler_target[2] = NAN;
			yap.attitude_controll.set_euler_target(euler_target);
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
