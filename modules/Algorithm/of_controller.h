#pragma once

#include <stdint.h>

class OpticalFlowController
{
public:
	OpticalFlowController();
	~OpticalFlowController();

	int reset();

	// unit:meter/s for flow, radian for lean angles
	// axis: positive for moving forward or left
	int update_controller(float flow_roll, float flow_pitch, float user_roll, float user_pitch, float dt);

	int get_result(float *result_roll, float *result_pitch);		// unit: radian

protected:	
	float m_error[2][3];	// [roll, pitch][p, i, d], unit: [roll, pitch][meter, meter*s, meter/s], P is integration of flow
	float m_result[2];		// unit: radian
	float m_user[2];		// unit: radian
};