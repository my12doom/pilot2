#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

class motion_detector
{
public:
	motion_detector()
	{
		threshold = 1;
		avg_count = -1;
	}
	~motion_detector()
	{

	}

	// reset everything but threshold.
	void reset()
	{
		avg_count = 0;
		memset(&accumulated, 0, sizeof(accumulated));
		memset(&sq_accumulated, 0, sizeof(sq_accumulated));
	}

	// ..
	void set_threshold(float threshold)
	{
		this->threshold = threshold;
	}

	// return true if motion detected.
	bool new_data(float *v)
	{
		if (avg_count <= 0)
		{
			memcpy(accumulated, v, sizeof(accumulated));
			for(int i=0; i<3; i++)
				sq_accumulated[i] = v[i]*v[i];
			avg_count = 1;

			return false;
		}

		// t = accumulated/avg_count - v
		float e[3];
		memcpy(e, accumulated, sizeof(e));
		for(int i=0; i<3; i++)
		{
			e[i] /= avg_count;
			e[i] -= v[i];
		}
		
		if (e[0]*e[0]+e[1]*e[1]+e[2]*e[2] > threshold*threshold)
		{
			memcpy(accumulated, v, sizeof(accumulated));
			for(int i=0; i<3; i++)
				sq_accumulated[i] = v[i]*v[i];
			avg_count = 1;

			return true;
		}

		for(int i=0; i<3; i++)
		{
			accumulated[i] += v[i];
			sq_accumulated[i] += v[i]*v[i];
		}

		avg_count++;

		return false;
	}

	// return number of data points averaged. the vector pointer remains untouched if data points count is 0.
	int get_average(float *avg, float *uncertainty = NULL)
	{
		if (avg_count <= 0)
			return 0;

		if (avg)
		{
			memcpy(avg, accumulated, sizeof(accumulated));
			for(int i=0; i<3; i++)
				avg[i] /= avg_count;
		}

		if (uncertainty)
		{
			for(int i=0; i<3; i++)
			{
				double avg = (double)accumulated[i] / avg_count;
				//std_variance[i] = sqrt(sq_accumulated[i]/avg_count - avg*avg);
				uncertainty[i] = sqrt(((double)sq_accumulated[i]/avg_count - avg*avg) / (avg_count-1));

			}
		}

		return avg_count;
	}
protected:
	int avg_count;
	float accumulated[3];
	float sq_accumulated[3];
	float threshold;
};
