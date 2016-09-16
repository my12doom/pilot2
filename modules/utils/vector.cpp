#include "vector.h"
#include <Protocol/common.h>
#include <math.h>

// http://zh.wikipedia.org/wiki/%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5
// our plane header direction is y+ axis, right wing is x+ axis, z+ is the sky direction
// theirs: header direction is z+ axis, right wing is x+ axis, y+ is the sky direction
void vector_rotate_roll(vector *v, float delta)
{
	vector v_tmp = *v;

	float sinX = sin(delta);
	float cosX = sqrt(1-sinX*sinX);


	v->V.x = v_tmp.V.x * (+cosX) + v_tmp.V.z * (+sinX);
	v->V.y = v_tmp.V.y;
	v->V.z = v_tmp.V.x * (-sinX) + v_tmp.V.z * (+cosX);
}
void vector_rotate_pitch(vector *v, float delta)
{
	vector v_tmp = *v;

	float sinX = sin(delta);
	float cosX = sqrt(1-sinX*sinX);


	v->V.x = v_tmp.V.x;
	v->V.y = v_tmp.V.y * (+cosX) + v_tmp.V.z * (+sinX);
	v->V.z = v_tmp.V.y * (-sinX) + v_tmp.V.z * (+cosX);
}
void vector_rotate_yaw(vector *v, float delta)
{
	vector v_tmp = *v;

	float sinX = sin(delta);
	float cosX = sqrt(1-sinX*sinX);

	v->V.x = v_tmp.V.x * (+cosX) + v_tmp.V.y * (-sinX);
	v->V.y = v_tmp.V.x * (+sinX) + v_tmp.V.y * (+cosX);
	v->V.z = v_tmp.V.z;
}

void vector_add(vector *a, vector *b)		// a = a+b
{
	int i;
	for (i=0; i<3; i++)
		a->array[i] += b->array[i];
}
void vector_sub(vector *a, vector *b)		// a = a-b
{
	int i;
	for (i=0; i<3; i++)
		a->array[i] -= b->array[i];
}
void vector_divide(vector *a, float b)		// a = a/b
{
	int i;
	for (i=0; i<3; i++)
		a->array[i] /= b;
}
void vector_multiply(vector *a, float b)	// a = a*b
{
	int i;
	for (i=0; i<3; i++)
		a->array[i] *= b;
}
void vector_rotate(vector *v, float *delta)
{
// vector v_tmp = *v;
// v->V.z -= delta[0]  * v_tmp.V.x + delta[1] * v_tmp.V.y;
// v->V.x += delta[0]  * v_tmp.V.z - delta[2]   * v_tmp.V.y;
// v->V.y += delta[1] * v_tmp.V.z + delta[2]   * v_tmp.V.x;
// 
// vector_multiply(v, vector_length(&v_tmp) / vector_length(v));

	vector_rotate_roll(v, delta[0]);
	vector_rotate_pitch(v, delta[1]);
	vector_rotate_yaw(v, delta[2]);
}
void vector_rotate2(vector *v, float *delta)
{
	vector_rotate_roll(v, -delta[0]);
	vector_rotate_pitch(v, -delta[1]);
	vector_rotate_yaw(v, -delta[2]);
}

void vector_normalize(vector *v)
{
	float l = vector_length(v);
	v->array[0] /= l;
	v->array[1] /= l;
	v->array[2] /= l;
}

float vector_length(vector *v)
{
	return sqrt(v->array[0]*v->array[0] + v->array[1] * v->array[1] + v->array[2] * v->array[2]);
}

float vector_angle(vector *v1, vector *v2)
{
	return (v1->V.x * v2->V.x + v1->V.y * v2->V.y + v1->V.z * v2->V.z)/vector_length(v1)/vector_length(v2);
}

int accel_vector_to_euler_angle(vector accel, vector *ouler)		// ouler angle roll, pitch are stored in ouler->array[0,1], yaw not calculated
{
	ouler->array[0] = radian_add(atan2(accel.V.x, accel.V.z), PI);
	ouler->array[1] = atan2(accel.V.y, (accel.V.z > 0 ? 1 : -1) * sqrt(accel.V.x*accel.V.x + accel.V.z * accel.V.z));
	ouler->array[1] = radian_add(ouler->array[1], PI);

	return 0;
}

vector vector_delta_angle(vector v1, vector v2)
{
	vector o;

	vector_normalize(&v1);
	vector_normalize(&v2);

	o.V.y = asin(v1.V.y * v2.V.z - v1.V.z * v2.V.y);
	o.V.x = asin(v1.V.x * v2.V.z - v1.V.z * v2.V.x);
	o.V.z = asin(v1.V.y * v2.V.x - v1.V.x * v2.V.y);

	return o;
}