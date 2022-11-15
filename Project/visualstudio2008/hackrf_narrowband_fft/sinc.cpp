#include "sinc.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "filter1d.h"

static float PI = acos(-1.0f);

void test_sinc()
{
	int count = 16384;
	float interpolation_factor = 0.3;
	int out_count = count * interpolation_factor;
	float sinc_range = 55.5f;

	fftwf_complex * fft = (fftwf_complex *)fftw_malloc(sizeof(fftwf_complex)*out_count);

	float *input = new float[count];
	for(int i=0; i<count; i++)
		input[i] = i < count/2 ? 0 : 1;

	input[count/4] = 1;

	FILE * in = fopen("D:\\temp\\tst.pcm", "rb");
	fread(input, 4, count, in);
	fclose(in);



	float *interpolated = new float[out_count];

	sinc_intepolator sinc;
	sinc.set_size(count, out_count);
	sinc.apply(input, interpolated);


// 	fftwf_plan p1 = fftwf_plan_dft_r2c_1d(count, input, fft, FFTW_ESTIMATE );
//  	fftwf_plan p2 = fftwf_plan_dft_c2r_1d(out_count, fft, interpolated, FFTW_ESTIMATE );
// 
// 	fftwf_execute(p1);
// 	memset(fft+count/2+1, 0, sizeof(fftwf_complex)*((out_count - count) / 2));
// 	fftwf_execute(p2);
// 
// 	fftwf_destroy_plan(p1);
// 	fftwf_destroy_plan(p2);
// 
// 	for(int i=0; i<out_count; i++)
// 		interpolated[i] /= count;

	FILE * out = fopen("D:\\temp\\float.pcm", "wb");
	fwrite(interpolated, 4, count*interpolation_factor, out);
	fclose(out);

	out = fopen("D:\\temp\\step.pcm", "wb");
	fwrite(input, 4, count, out);
	fclose(out);

// 	out = fopen("D:\\temp\\taps.pcm", "wb");
// 	fwrite(taps, 4, taps_count, out);
// 	fclose(out);

	delete input;
	delete interpolated;

	fftwf_free(fft);
}


sinc_intepolator::sinc_intepolator()
{
	fft = plan[0] = plan[1] = NULL;
	in_size = out_size = 0;
}
sinc_intepolator::~sinc_intepolator()
{
	if (fft)
		fftwf_free(fft);
	if (plan[0])
		fftwf_destroy_plan((fftwf_plan)plan[0]);
	if (plan[1])
		fftwf_destroy_plan((fftwf_plan)plan[1]);
}


int sinc_intepolator::set_size(int in_size, int out_size)
{
	if (this->in_size == in_size && this->out_size == out_size)
		return 0;

	if (fft)
		fftwf_free(fft);
	if (plan[0])
		fftwf_destroy_plan((fftwf_plan)plan[0]);
	if (plan[1])
		fftwf_destroy_plan((fftwf_plan)plan[1]);

	this->in_size = in_size;
	this->out_size = out_size;
	int maxsize = max(in_size, out_size);

	fft = fftw_malloc(sizeof(fftwf_complex)*maxsize);

	fftwf_complex * _fft = (fftwf_complex *)fft;

	float *tmp = new float[maxsize];
	memset(tmp, 0, sizeof(float) * maxsize);

	plan[0] = fftwf_plan_dft_r2c_1d(in_size, tmp, _fft, FFTW_ESTIMATE );
	plan[1] = fftwf_plan_dft_c2r_1d(out_size, _fft, tmp, FFTW_ESTIMATE );

	delete tmp;

	return 0;
}

int sinc_intepolator::apply(float *in, float *out)
{
	fftwf_execute_dft_r2c((fftwf_plan)plan[0], in, (fftwf_complex*)fft);
	if (out_size > in_size)
		memset((fftwf_complex*)fft+in_size/2+1, 0, sizeof(fftwf_complex)*((out_size - in_size) / 2));
	fftwf_execute_dft_c2r((fftwf_plan)plan[1], (fftwf_complex*)fft, out);

	for(int i=0; i<out_size; i++)
		out[i] /= in_size;

	return 0;
}
