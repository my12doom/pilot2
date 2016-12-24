#pragma once

#include <string.h>

namespace math
{

template<int order>
class FIR
{
public:
	FIR(const float *weight)
	{
		memcpy(weights, weight, sizeof(weights));
		memset(delay_elements, 0, sizeof(delay_elements));
	}
	~FIR()
	{
	}

	float apply(float new_value)
	{
		memmove(delay_elements, delay_elements + 1, sizeof(delay_elements) - 4);

		delay_elements[order-1] = new_value;
		
		float o = 0;
		for(int i=0; i<order; i++)
			o += delay_elements[i] * weights[i];

		return o;
	}

protected:
	float delay_elements[order];
	float weights[order];
};
}
