#include "mode_of_loiter.h"
#include "pilot.h"
#include <utils/param.h>
#include <utils/log.h>

static param quadcopter_range[3] = 
{
	param("rngR", PI / 5),			// roll
	param("rngP", PI / 5),			// pitch
	param("rngY", PI / 8),			// yaw
};

int flight_mode_of_loiter::setup()
{
	yap.alt_controller.reset();
	
	return 0;
}
int flight_mode_of_loiter::exit()
{
	return 0;
}
int flight_mode_of_loiter::loop(float dt)
{
	// altitude
	yap.default_alt_controlling();

	// attitude
	// airborne or armed and throttle up
	bool after_unlock_action = yap.airborne || yap.rc[2] > 0.55f;
	if (after_unlock_action)	// airborne or armed and throttle up
	{
		float stick_roll = yap.rc[0] * quadcopter_range[0];
		float stick_pitch = -yap.rc[1] * quadcopter_range[1];	// pitch stick and coordinate are reversed

		float flow_roll = -yap.frame.flow_comp_m_x/1000.0f * 10;
		float flow_pitch = -yap.frame.flow_comp_m_y/1000.0f * 10;

		float pixel_compensated_x = yap.frame.pixel_flow_x_sum - yap.body_rate.array[0] * 18000 / PI * 0.0028f;
		float pixel_compensated_y = yap.frame.pixel_flow_y_sum - yap.body_rate.array[1] * 18000 / PI * 0.0028f;

		float wx = pixel_compensated_x / 28.0f * 100 * PI / 180;
		float wy = pixel_compensated_y / 28.0f * 100 * PI / 180;

		float vx = wx * yap.frame.ground_distance/1000.0f * 1.15f;
		float vy = wy * yap.frame.ground_distance/1000.0f * 1.15f;


		//transform fused velocity from ned to body 
		float v_flow_body[3];
		yap.ekf_est.tf_ned2body(yap.v_flow_ned,v_flow_body);
		vx=-v_flow_body[1];
		vy=v_flow_body[0];

		yap.of_controller.update_controller(vx, vy, stick_roll, stick_pitch, dt);
		float euler_target[3] = {0,0, NAN};
		yap.of_controller.get_result(&euler_target[0], &euler_target[1]);
		yap.attitude_controll.set_euler_target(euler_target);

		float pixel_compensated[6] = {pixel_compensated_x, pixel_compensated_y, wx, wy, vx, vy};
		log2(pixel_compensated, 10, sizeof(pixel_compensated));

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
