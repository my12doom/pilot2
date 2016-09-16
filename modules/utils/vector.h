#ifndef __VECTOR_H__
#define __VECTOR_H__


typedef union
{
	float array[3];
	struct
	{
		float x;
		float y;
		float z;
	}V;
}vector;
	
void vector_add(vector *a, vector *b);		// a = a+b
void vector_sub(vector *a, vector *b);		// a = a-b
void vector_divide(vector *a, float b);		// a = a/b
void vector_multiply(vector *a, float b);	// a = a*b
void vector_rotate(vector *v, float *delta);
void vector_rotate2(vector *v, float *delta);
float vector_length(vector *v);
float vector_angle(vector *v1, vector *v2);	// return cos(angle(v1, v2))
void vector_normalize(vector *v);
int accel_vector_to_euler_angle(vector accel, vector *ouler);		// ouler angle roll, pitch are stored in ouler->array[0,1], yaw not calculated
vector vector_delta_angle(vector v1, vector v2);

#endif
