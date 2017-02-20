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
	yap.of_controller.reset();
	
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
		float att[2];
		yap.attitude_controll.get_attitude_from_stick(yap.rc, att);

		float wx = yap.flow.x - yap.body_rate.array[0];
		float wy = yap.flow.y - yap.body_rate.array[1];
		
		float vx = wx * isnan(yap.sonar_distance) ? 1 : yap.sonar_distance;
		float vy = wy * isnan(yap.sonar_distance) ? 1 : yap.sonar_distance;
		
		//transform fused velocity from ned to body 
		float v_flow_body[3];
		yap.ekf_est.tf_ned2body(yap.v_flow_ned,v_flow_body);
		//vx=-v_flow_body[1];
		//vy=v_flow_body[0];

		if (yap.flow.quality < 0.45)
		{
			vx = vy = 0;
		}

		yap.of_controller.update_controller(vx, vy, att[0], att[1], dt);
		float euler_target[3] = {0,0, NAN};
		yap.of_controller.get_result(&euler_target[0], &euler_target[1]);
		yap.attitude_controll.set_euler_target(euler_target);

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
