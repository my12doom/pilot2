#pragma once
#include <stdint.h>
#include <utils/vector.h>

class motion_detector
{
public:
	motion_detector(int dimension = 3);
	~motion_detector();

	// reset everything but threshold.
	void reset();

	// ..
	void set_threshold(float threshold);

	// return true if motion detected.
	bool new_data(vector v);

	// return number of data points averaged. the vector pointer remains untouched if data points count is 0.
	int get_average(vector *out);
protected:
	int dimension;
	int avg_count;
	vector accumulated;
	float threshold;
};
