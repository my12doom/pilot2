#ifndef __VECTOR_H__
#define __VECTOR_H__


typedef union _vector
{
	float array[3];
	struct
	{
		float x;
		float y;
		float z;
	}V;

	float & operator [](int index){return array[index];}
	void operator +=(const union _vector &v);
	union _vector operator +(const union _vector &v);
	void operator -=(const union _vector &v);
	union _vector operator -(const union _vector &v);
	void operator *=(const float &v);
	union _vector operator *(const float &v);
	void operator /=(const float &v);
	union _vector operator /(const float &v);

	float length();
	float dot(const union _vector &v);
	float angle(const union _vector &v);
	void normalize();
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

#endif
