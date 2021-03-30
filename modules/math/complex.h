#pragma once

#include <math.h>
#include <stdint.h>

class complex
{
public:
	complex()
	{
		this->real = 0;
		this->image = 0;
	}
	complex(float real, float image)
	{
		this->real = real;
		this->image = image;
	}
	complex(const complex &v)
	{
		this->real = v.real;
		this->image = v.image;
	}
	static complex from_phase_magnitude(float phase, float magnitude)
	{
		complex o;
		o.real = cos(phase) * magnitude;
		o.image = sin(phase) * magnitude;
		return o;
	}
	~complex()
	{

	}

	float real;
	float image;
	float sq_magnitude() 
	{
		return real*real+image*image;
	}
	float magnitude() 
	{
		return sqrt(real*real+image*image);
	}
	float argument() 
	{
		return atan2(image, real);
	}
	complex conjugate() 
	{
		complex o(real, -image);
		return o;
	}

	complex operator *(const complex &b)
	{
		complex o;

		o.real = this->real * b.real - this->image * b.image;
		o.image = this->real * b.image + this->image * b.real;

		return o;
	}

	complex operator *(float b)
	{
		complex o;

		o.real = this->real * b;
		o.image = this->image * b;

		return o;
	}

	complex operator *=(const complex &b)
	{
		complex o;

		o.real = this->real * b.real - this->image * b.image;
		o.image = this->real * b.image + this->image * b.real;

		*this = o;
		return o;
	}

	complex operator +(const complex &b)
	{
		complex o(real, image);

		o.real += b.real;
		o.image += b.image;

		return o;
	}

	complex& operator +=(const complex &b)
	{
		real += b.real;
		image += b.image;

		return *this;
	}

	complex operator -(const complex &b)
	{
		complex o(real, image);

		o.real -= b.real;
		o.image -= b.image;

		return o;
	}

	complex& operator -=(const complex &b)
	{
		real -= b.real;
		image -= b.image;

		return *this;
	}


	complex operator /(const float &b)
	{
		complex o(real, image);

		o.real /= b;
		o.image /= b;

		return o;
	}

	complex operator =(const float &b)
	{
		this->real = b;
		this->image = 0;

		return *this;
	}

	complex operator / (complex &b)
	{
		return ((*this) * b.conjugate()) / b.sq_magnitude();
	}
};

template<class ValueType, int q_number>
class complexQ
{
public:
	complexQ()
	{
		this->real = 0;
		this->image = 0;
	}
	complexQ(ValueType real, ValueType image)
	{
		this->real = real;
		this->image = image;
	}
	complexQ(const complexQ &v)
	{
		this->real = v.real;
		this->image = v.image;
	}
	static complexQ from_phase_magnitude(ValueType phase, ValueType magnitude)
	{
		complexQ o;
		o.real = cos((float)phase/ONE) * magnitude;
		o.image = sin((float)phase/ONE) * magnitude;
		return o;
	}
	~complexQ()
	{

	}

	ValueType real;
	ValueType image;
	static const ValueType ONE = (1 << q_number) -1;
	ValueType sq_magnitude() 
	{
		return (real*real+image*image)>>q_number;
	}
	ValueType magnitude() 
	{
		return sqrt((float)real*real+image*image);
	}
	ValueType argument()
	{
		static float __PI = acos(-1.0);
		return atan2((float)image, real) / __PI * ONE;
	}
	complexQ conjugate() 
	{
		complexQ o(real, -image);
		return o;
	}

	complexQ operator *(const complexQ &b)
	{
		complexQ o;

		o.real = (this->real * b.real - this->image * b.image) >> q_number;
		o.image = (this->real * b.image + this->image * b.real) >> q_number;

		return o;
	}

	complexQ operator *(ValueType b)
	{
		complexQ o;

		o.real = this->real * b;
		o.image = this->image * b;

		return o;
	}

	complexQ operator *=(const complexQ &b)
	{
		complexQ o;

		o.real = (this->real * b.real - this->image * b.image) >> q_number;
		o.image = (this->real * b.image + this->image * b.real) >> q_number;

		*this = o;
		return o;
	}

	complexQ operator +(const complexQ &b)
	{
		complexQ o(real, image);

		o.real += b.real;
		o.image += b.image;

		return o;
	}

	complexQ& operator +=(const complexQ &b)
	{
		real += b.real;
		image += b.image;

		return *this;
	}

	complexQ operator -(const complexQ &b)
	{
		complexQ o(real, image);

		o.real -= b.real;
		o.image -= b.image;

		return o;
	}

	complexQ& operator -=(const complexQ &b)
	{
		real -= b.real;
		image -= b.image;

		return *this;
	}


	complexQ operator /(const ValueType &b)
	{
		complexQ o(real, image);

		o.real = (o.real << q_number) / b;
		o.image = (o.image << q_number) / b;

		return o;
	}

	complexQ operator =(const ValueType &b)
	{
		this->real = b;
		this->image = 0;

		return *this;
	}

	complexQ operator / (complexQ &b)
	{
		return ((*this) * b.conjugate()) / b.sq_magnitude();
	}
};


typedef complexQ<int16_t, 15> complexQ15;