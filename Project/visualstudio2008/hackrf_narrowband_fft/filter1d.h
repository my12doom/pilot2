#pragma once

#include <stdint.h>
#include <stdlib.h>
#include "autolock.h"

class filter1d
{
public:
    filter1d();
    filter1d(float *taps, int taps_count);
    ~filter1d();

    int set_taps(float *taps, int taps_count);
	int set_taps_fft(float *taps, int taps_count, int data_count);
    int apply(float *in, int intput_count, float *out = NULL, int output_count = -1);

protected:
    float *taps;
    int taps_count;

	bool use_fft;
	double *taps_fft;
	double *data_fft;
	void *fftw_plans[2];
	int N;
	_critical_section cs;
};
