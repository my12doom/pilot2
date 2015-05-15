#pragma once

#include <stdint.h>

class altitude_estimator
{
public:
	altitude_estimator();
	~altitude_estimator();

	float state[4];	// 4x1 matrix, altitude, climb, accel, accel_bias

	// pass NAN to baro to indicate that baro data is not ready.
	int update(float accelz, float baro, float dt);

	// set to true to lower baro factor for land effect compensating
	void set_land_effect(bool compensate_land_effect){this->compensate_land_effect = compensate_land_effect;}

	// set static_mode to true to tell the estimator that the machine is not flying, and estimator should trust more baro data.
	void set_static_mode(bool static_mode);

protected:
	
	bool compensate_land_effect;
	void matrix_error(const char*msg);
	void matrix_mov(float *dst, const float *src, int row, int column);
	void matrix_add(float *op1, float *op2, int row, int column);
	void matrix_sub(float *op1, float *op2, int row, int column);
	int matrix_mul(float *out, const float *m1, int row1, int column1, const float *m2, int row2, int column2);
	int inverse_matrix2x2(float *m);
};