#include "altitude_estimatorCF.h"
#include <string.h>
#include <Protocol/common.h>

float _time_constant_z = 5.0f;
float _k1_z = 3 / _time_constant_z;
float _k2_z = 3 / (_time_constant_z*_time_constant_z);
float _k3_z = 1 / (_time_constant_z*_time_constant_z*_time_constant_z);


altitude_estimatorCF::altitude_estimatorCF()
:compensate_land_effect(false)
{
	state[0] = 0;
	state[1] = 0;
	state[2] = 0;
	state[3] = 0;
	_position_error = 0;
	_position_base = 0;
	_position_correction = 0;
	_velocity_base = 0;
	_velocity_correction = 0;
}

altitude_estimatorCF::~altitude_estimatorCF()
{

}

int altitude_estimatorCF::update(float accelz, float baro, float dt)
{
	if (dt > 0.2f)
		return -1;

	if (!isnan(baro))
	{
		_position_error = baro - (_position_base + _position_correction);
	}

	state[2] = accelz;
	state[3] += _position_error * _k3_z  * dt;
	_velocity_correction += _position_error * _k2_z  * dt;
	_position_correction += _position_error * _k1_z  * dt;

	const float velocity_increase = (accelz + state[3]) * dt;
	_position_base += (_velocity_base + _velocity_correction + velocity_increase*0.5f) * dt;
	_velocity_base += velocity_increase;

	state[0] = _position_base + _position_correction;
	state[1] = _velocity_base + _velocity_correction;

	return 0;
}
