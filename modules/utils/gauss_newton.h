// sphere fitting of (b*(x+a))^2 + (d*(y+c))^2 + (f*(z+e))^2 = 1
// see https://chionophilous.wordpress.com/author/rolfeschmidt/ for detail.
// the original artical use definition | (x-b0)*b3, (y-b1)*b4, (z-b2)*b5 | = 1

#pragma once

#include <stdint.h>

class gauss_newton_sphere_fitting
{
public:
	gauss_newton_sphere_fitting();
	~gauss_newton_sphere_fitting();

	int calculate(const float *data, int vector_count);		// data is a array of [x, y, z] * vector_count floats
	int get_result(float *result);							// get the fitting result of sphere (ax+b)^2 + (cy+d)^2 + (ez+f) = 1
															// result[0--6] = [a, b, c, d, e, f]

protected:
	void calibrate_model_matrices();
	void find_delta();
	void calibrate_model();
	void reset_calibration_matrices();
	void update_calibration_matrices(const float* data);


	float beta[6];						//parameters for model.  beta[0], beta[1], and beta[2] are the 0-G marks (about 512),
										// while beta[3], beta[4], and beta[5] are the scaling factors.  So, e.g., if xpin reads
										// value x, number of G's in the x direction in beta[3]*(x - beta[0]).

	//matrices for Gauss-Newton computations
	float JS[6][6];
	float dS[6];
	float delta[6];
};
