// sphere fitting of (b*(x+a))^2 + (d*(y+c))^2 + (f*(z+e))^2 = 1
// see https://chionophilous.wordpress.com/author/rolfeschmidt/ for detail.
// the original artical use definition | (x-b0)*b3, (y-b1)*b4, (z-b2)*b5 | = 1

#include "gauss_newton.h"
#include <stdio.h>
#include <math.h>

#if !defined (__ARMCC_VERSION) || (__ARMCC_VERSION < 6010050)
static inline float fmin(float a, float b)
{
	return a>b?b:a;
}
static inline float fmax(float a, float b)
{
	return a>b?a:b;
}
#endif
gauss_newton_sphere_fitting::gauss_newton_sphere_fitting()
{

}

gauss_newton_sphere_fitting::~gauss_newton_sphere_fitting()
{

}

int gauss_newton_sphere_fitting::get_result(float *result)		// get the fitting result of sphere (ax+b)^2 + (cy+d)^2 + (ez+f) = 1
																// result[0--6] = [a, b, c, d, e, f]
{
	result[0] = -beta[0];
	result[1] = -beta[1];
	result[2] = -beta[2];
	result[3] = beta[3];
	result[4] = beta[4];
	result[5] = beta[5];

	return 0;
}

//Gauss-Newton functions

void gauss_newton_sphere_fitting::reset_calibration_matrices() {
	int j,k;
	for(j=0;j<6;++j) {
		dS[j] = 0.0;
		for(k=0;k<6;++k) {
			JS[j][k] = 0.0;
		}
	}

}

void gauss_newton_sphere_fitting::update_calibration_matrices(const float* data) {
	int j, k;
	float dx, b;
	float residual = 1.0;
	float jacobian[6];

	for(j=0;j<3;++j) {
		b = beta[3+j];
		dx = data[j] - beta[j];
		residual -= b*b*dx*dx;
		jacobian[j] = 2.0f*b*b*dx;
		jacobian[3+j] = -2.0f*b*dx*dx;
	}

	for(j=0;j<6;++j) {
		dS[j] += jacobian[j]*residual;
		for(k=0;k<6;++k) {
			JS[j][k] += jacobian[j]*jacobian[k];
		}
	}

}



int gauss_newton_sphere_fitting::calculate(const float *data, int vector_count)					// data is 3*vector_count floats
{
	int i, j, k;
	float dx, b;
	float eps = 0.000000001;
	int total_num_iterations = 20;
	int num_iterations = total_num_iterations;

	// initialize beta to resonable value
	float min_value[3] = {99999, 99999, 99999};
	float max_value[3] = {-99999, -99999, -99999};
	for(int i=0; i<vector_count; i++)
	{
		for(int j=0; j<3; j++)
		{
			min_value[j] = fmin(min_value[j], data[3*i+j]);
			max_value[j] = fmax(max_value[j], data[3*i+j]);
		}
	}

	for(int j=0; j<3; j++)
	{
		beta[j] = (max_value[j] + min_value[j]) / 2;
		beta[j+3] = 2 / (max_value[j] - min_value[j]);
	}

	// iterate
	while (--num_iterations >=0)
	{
		reset_calibration_matrices();
		for(i=0;i<vector_count; i++)
			update_calibration_matrices(data+3*i);
		find_delta();

		float change = delta[0]*delta[0] + delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3]/(beta[3]*beta[3]) + delta[4]*delta[4]/(beta[4]*beta[4]) + delta[5]*delta[5]/(beta[5]*beta[5]); 

		for(i=0;i<6;++i)
			beta[i] -= delta[i];

// 		printf( "Num iterations: %d", total_num_iterations - num_iterations);
// 		printf( " change:(%.2f,%.2f,%.2f) %f\n", delta[0], delta[1], delta[2], change);

		if (change < eps)
			break;
	}

	return 0;
}

void gauss_newton_sphere_fitting::find_delta() {
	//Solve 6-d matrix equation JS*x = dS
	//first put in upper triangular form
	int i,j,k;
	float mu;

	//make upper triangular
	for(i=0;i<6;++i) {
		//eliminate all nonzero entries below JS[i][i]
		for(j=i+1;j<6;++j) {
			mu = JS[i][j]/JS[i][i];
			if(mu != 0.0f) {
				dS[j] -= mu*dS[i];
				for(k=j;k<6;++k) {
					JS[k][j] -= mu*JS[k][i];
				} 
			}
		}
	}

	//back-substitute
	for(i=5;i>=0;--i) {
		dS[i] /= JS[i][i];
		JS[i][i] = 1.0;
		for(j=0;j<i;++j) {
			mu = JS[i][j];
			dS[j] -= mu*dS[i];
			JS[i][j] = 0.0;
		}
	}

	for(i=0;i<6;++i) {
		delta[i] = dS[i];
	}
}
