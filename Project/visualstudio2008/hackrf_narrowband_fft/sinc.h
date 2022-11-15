#pragma once

#include <stdint.h>
#include "fftw/fftw3.h"

class sinc_intepolator
{
public:
	sinc_intepolator();
	~sinc_intepolator();

	int set_size(int in_size, int out_size);
	int apply(float *in, float *out);

protected:
	fftwf_plan plan[2];
	void *fft;
	int in_size;
	int out_size;
};

void test_sinc();
