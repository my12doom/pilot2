#include "motion_detector.h"
#include <string.h>

motion_detector::motion_detector(int dimension /* = 3 */)
{
	threshold = 1;
	reset();
}

motion_detector::~motion_detector()
{
	
}

void motion_detector::reset()
{
	avg_count = 0;
	memset(&accumulated, 0, sizeof(accumulated));
}

void motion_detector::set_threshold(float threshold)
{
	this->threshold = threshold;
}

// return true if motion detected.
bool motion_detector::new_data(vector v)
{
	if (avg_count <= 0)
	{
		accumulated = v;
		avg_count = 1;

		return false;
	}

	// t = accumulated/avg_count - v
	vector t = accumulated;
	vector_divide(&t, avg_count);
	vector_sub(&t, &v);
	
	if (vector_length(&t) > threshold)
	{
		accumulated = v;
		avg_count = 1;

		return true;
	}

	vector_add(&accumulated, &v);
	avg_count++;

	return false;
}

// return number of data points averaged. the vector pointer remains untouched if data points count is 0.
int motion_detector::get_average(vector *out)
{
	if (avg_count <= 0)
		return 0;

	if (out)
	{
		*out = accumulated;
		vector_divide(out, avg_count);
	}

	return avg_count;
}
