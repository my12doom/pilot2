#include "filter1d.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "fftw/fftw3.h"

filter1d::filter1d()
{
    taps = NULL;
	taps_fft = NULL;
	data_fft = NULL;
	fftw_plans[0] = NULL;
	fftw_plans[1] = NULL;
}

filter1d::filter1d(float *taps, int taps_count)
{
    filter1d();
    set_taps(taps, taps_count);
}

filter1d::~filter1d()
{
	_autolock lck(&cs);
    if (taps)
        delete [] taps;

	if (taps_fft)
		fftw_free(taps_fft);
	if (data_fft)
		fftw_free(data_fft);
	if (fftw_plans[0])
		fftw_destroy_plan((fftw_plan)fftw_plans[0]);
	if (fftw_plans[1])
		fftw_destroy_plan((fftw_plan)fftw_plans[1]);

	use_fft = true;
}

int filter1d::set_taps(float *taps, int taps_count)
{
	_autolock lck(&cs);
    if (this->taps)
        delete [] this->taps;

    this->taps = new float[taps_count];
    memcpy(this->taps, taps, taps_count * sizeof(float));
    this->taps_count = taps_count;

	use_fft = false;

    return 0;
}

int filter1d::set_taps_fft(float *taps, int taps_count, int data_count)
{
	_autolock lck(&cs);
	if (taps_fft)
		fftw_free(taps_fft);
	if (data_fft)
		fftw_free(data_fft);
	if (fftw_plans[0])
		fftw_destroy_plan((fftw_plan)fftw_plans[0]);
	if (fftw_plans[1])
		fftw_destroy_plan((fftw_plan)fftw_plans[1]);

	// normalize
// 	float total = 0;
// 	for(int i=0; i<taps_count; i++)
// 		total += taps[i]*taps[i];
// 	total = sqrt(total);
// 	for(int i=0; i<taps_count; i++)
// 		taps[i] /= total;

	// normalize
	float total = 0;
	for(int i=0; i<taps_count; i++)
		total += taps[i];
	for(int i=0; i<taps_count; i++)
		taps[i] /= total;


	int len = data_count + taps_count + 16*2;		// 16*2: 2*SIMD padding
	N = 2;
	while(N<len)
		N<<=1;

	taps_fft = (double*)fftw_malloc(sizeof(fftw_complex)*N);
	data_fft = (double*)fftw_malloc(sizeof(fftw_complex)*N);

	fftw_plans[0] = fftw_plan_dft_1d(N, (fftw_complex*)taps_fft, (fftw_complex*)taps_fft, FFTW_FORWARD, FFTW_ESTIMATE);
	fftw_plans[1] = fftw_plan_dft_1d(N, (fftw_complex*)data_fft, (fftw_complex*)data_fft, FFTW_BACKWARD, FFTW_ESTIMATE);

	memset(taps_fft, 0, sizeof(fftw_complex)*N);

	int half_taps = taps_count/2;
	int pos_part = taps_count - half_taps;
	int neg_part = taps_count - pos_part;
	for(int i=0; i<pos_part; i++)
		taps_fft[i*2] = taps[i+half_taps];
	for(int i=0; i<neg_part; i++)
		taps_fft[(N-neg_part+i)*2] = taps[i];

	fftw_execute((fftw_plan)fftw_plans[0]);

	use_fft = true;
	return 0;
}

int filter1d::apply(float *in, int intput_count, float *out /*= NULL */, int output_count /*= -1*/)
{
	_autolock lck(&cs);
	if (!use_fft)
	{
		if (output_count <= 0)
			output_count = intput_count;

		if (in == out)
			out = NULL;

		float *p = out;
		if (!out)
			p = new float[output_count];

		int half_tap = (taps_count - 1) / 2;
		if (intput_count == output_count)
		{
			for(int i=0; i<output_count; i++)
			{
				p[i] = 0;
				for(int j=0; j<taps_count; j++)
				{
					int idx = j - half_tap + i;
					if (idx < 0 || idx >= intput_count)
						continue;
					p[i] += taps[taps_count-1-j] * in[idx];
				}
			}
		}
		
		if (!out)
		{
			memcpy(in, p, sizeof(float)*output_count);
			delete [] p;
		}

		return 0;
	}

	else
	{
		if (output_count == -1)
			output_count = intput_count;
		if (intput_count != output_count || intput_count > (N+16*2))
			return -1;

		// copy data in
		memset(data_fft, 0, sizeof(fftw_complex)*N);
		for(int i=0; i<intput_count; i++)
			data_fft[i<<1] = in[i];

		// FFT
		fftw_execute_dft((fftw_plan)fftw_plans[0], (fftw_complex*)data_fft, (fftw_complex*)data_fft);
		
		// complex point-wise multiply
		for(int i=0; i<N; i++)
		{
			double real = data_fft[(i<<1)+0] * taps_fft[(i<<1)+0] - data_fft[(i<<1)+1] * taps_fft[(i<<1)+1];
			double image = data_fft[(i<<1)+0] * taps_fft[(i<<1)+1] + taps_fft[(i<<1)+0] * data_fft[(i<<1)+1];
			
			data_fft[i<<1] = real;
			data_fft[(i<<1)+1] = image;
		}

		// IFFT
		fftw_execute((fftw_plan)fftw_plans[1]);

		// copy data out (and scale)
		float *p = out ? out : in;
		double scale = 1.0/N;
		for(int i=0; i<intput_count; i++)
			p[i] = float(data_fft[i<<1] * scale);
	}

	// never here
	return -1;
}
