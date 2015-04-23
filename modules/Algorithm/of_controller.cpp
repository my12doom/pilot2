#include "of_controller.h"
#include <Protocol/common.h>

// APM default factor: 2.5, 0.5, 0.12, 100
// P=2.5: 100cm distance -> 2.5 degree angle
// I=0.5: 100cm*s -> 0.5 degree angle
// D=0.12: 100cm/s speed -> 0.12 degree angle
// IMAX: just a placebo

float pid_factor[2][4] =
{
	{2.7f, 0.2f, 1.5f, 15},
	{2.7f, 0.2f, 1.5f, 15},
};

OpticalFlowController::OpticalFlowController()
{

	for(int i=0; i<2; i++)
	{
		for(int j=0; j<3; j++)
		{
			pid_factor[i][j] *= 2 * PI / 180;
		}
	}
}

OpticalFlowController::~OpticalFlowController()
{

}

int OpticalFlowController::update_controller(float flow_roll, float flow_pitch, float stick_roll, float stick_pitch, float dt)			// unit: meter/s
{
	// save stick input
	m_user[0] = stick_roll;
	m_user[1] = stick_pitch;

	if ( (fabs(stick_roll) < 5 * PI / 180 ) && (fabs(stick_pitch) < 5 * PI / 180))
	{
		// integrate pos (also P)
		m_error[0][0] += flow_roll * dt;
		m_error[1][0] += flow_pitch * dt;

		// pid

		// I
		m_error[0][1] += m_error[0][0] *dt;
		m_error[1][1] += m_error[1][0] *dt;
		m_error[0][1] = limit(m_error[0][1], -pid_factor[0][3], pid_factor[0][3]);
		m_error[1][1] = limit(m_error[1][1], -pid_factor[1][3], pid_factor[1][3]);

		// D, with 5hz LPF
		if (isnan(m_error[0][2]))
		{
			m_error[0][2] = flow_roll;
			m_error[1][2] = flow_pitch;
		}
		else
		{
			float alpha = dt / (dt + 1.0f/(2 * PI * 5.0f));	
			m_error[0][2] = flow_roll * alpha + m_error[0][2] * (1-alpha);
			m_error[1][2] = flow_pitch * alpha + m_error[1][2] * (1-alpha);
		}

		// sum
		float new_roll =  m_error[0][0] * pid_factor[0][0] + m_error[0][1] * pid_factor[0][1] + m_error[0][2] * pid_factor[0][2];
		float new_pitch = m_error[1][0] * pid_factor[1][0] + m_error[1][1] * pid_factor[1][1] + m_error[1][2] * pid_factor[1][2];

		// limit rotation speed, 30 degree/s
		float delta_roll = new_roll - m_result[0];
		float delta_pitch = new_pitch - m_result[1];
		delta_roll = limit(delta_roll, -dt * 30 * PI / 180, dt * 30 * PI / 180);
		delta_pitch = limit(delta_pitch, -dt * 30 * PI / 180, dt * 30 * PI / 180);
		m_result[0] += delta_roll;
		m_result[1] += delta_pitch;

		// limit maximum flow angle: 10 degree
		m_result[0] = limit(m_result[0], -10 * PI / 180, 10 * PI / 180);
		m_result[1] = limit(m_result[1], -10 * PI / 180, 10 * PI / 180);
	}

	else
	{
		reset();
	}

	return 0;
}

int OpticalFlowController::reset()
{
	// pass through
	m_result[0] = 0;
	m_result[1] = 0;

	// reset pos integral and pid integral
	m_error[0][0] = 0;
	m_error[1][0] = 0;
	m_error[0][1] = 0;
	m_error[1][1] = 0;
	m_error[0][2] = NAN;
	m_error[1][2] = NAN;

	return 0;
}

int OpticalFlowController::get_result(float *result_roll, float *result_pitch)		// unit: radian
{
	*result_roll = m_result[0] + m_user[0];
	*result_pitch = m_result[1] + m_user[1];

	TRACE("\rof:%.2f,%.2f", *result_roll * 180 / PI, *result_pitch * 180 / PI);

	return 0;
}
