#include <Windows.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "hackrf.h"
#include "fftw/fftw3.h"
#include "Fourier.h"

#pragma comment(lib, "libhackrf.lib")
#pragma comment(lib, "libusb-1.0.lib")
#pragma comment(lib, "pthreadVSE2.lib")
#pragma comment(lib, "fftw/libfftw3f-3.lib")

hackrf_device* device = NULL;
#define N 65536

fftwf_complex * in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
fftwf_complex * out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
fftwf_plan p;

FILE * f = fopen("Z:\\log2.pcm", "wb");
FILE * csv = fopen("Z:\\spectrom.csv", "wb");

float amp[N] = {0};
int amp_counter = 0;
float center_frequency = 5725.0E6;
int rx_callback(hackrf_transfer* transfer)
{
	int8_t * buf = (int8_t *)transfer->buffer;
	int len = transfer->valid_length;

	float peak = 0;
	static float peak_lpf = 0;
	float alpha = 0.05;
	float DC = 0;
	float alpha_DC = 1e-3;

	for(int i=0; i<len; i++)
		DC = buf[i] * alpha_DC + DC * (1-alpha_DC);

	for(int i=0; i<len; i++)
	{
		float v = (buf[i] - DC)/128;
		peak = max(peak, v);
	}
	peak_lpf = peak * alpha + (1-alpha) * peak_lpf;
	printf("\rpeak=%03.2f/%03.2f             ", log10(peak)*40, log10(peak_lpf)*40);

	fwrite(buf, 1, len, f);
	for(int j=0; j<len/2/N; j++)
	{
		for(int i=0; i<N; i++)
		{
			in[i][0] = buf[i*2+0]/128.0f;
			in[i][1] = buf[i*2+1]/128.0f;
		}
		fftwf_execute(p);
		for(int i=1; i<N; i++)
		{
			float v = sqrt(out[i][0]*out[i][0] + out[i][1]*out[i][1]);
			amp[i] += v;
		}

		buf += N*2;
 		amp_counter++;

	}


	return 0;
}


int StartHackRF()
{
	int result;

	result = hackrf_init();
	if( result != HACKRF_SUCCESS ) {
		printf("init_error");
		return result;
	}

	result = hackrf_open(&device);
	if( result != HACKRF_SUCCESS ) {
		printf("device open_error");
		return result;
	}
	///////////////////////////////////
	//////参数设置部分////////
	result |= hackrf_set_amp_enable(device, 1);
	result |= hackrf_set_sample_rate(device, 20000000);
	result |= hackrf_set_baseband_filter_bandwidth(device, 25e6);
	result |= hackrf_set_vga_gain(device, 0); // step: 2db
	result |= hackrf_set_lna_gain(device, 40); // step: 8db
	result |= hackrf_set_freq(device, center_frequency);

	uint16_t v = 0;
	LARGE_INTEGER l1,l2,freq;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&l1);
	hackrf_max2837_read(device, 2, &v);
	QueryPerformanceCounter(&l2);
	float s = (float)(l2.QuadPart-l1.QuadPart) / (freq.QuadPart);

	result |= hackrf_start_rx(device, rx_callback, NULL);
	return result;
}


int main(int argc, char* argv[])
{
	p = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
	int res = StartHackRF();
	if (res != HACKRF_SUCCESS)
	{
		printf("%d:%s\n", res, hackrf_error_name((hackrf_error)res));
		return 0;
	}

	fprintf(csv, "N,freq,amp\n");
	int t = GetTickCount();
	while(hackrf_is_streaming(device) == HACKRF_TRUE)
	{
		Sleep(10);

// 		if (GetTickCount() - t > 1000)
// 			hackrf_stop_rx(device);
	}

	hackrf_exit();

	float v = log10(127.0)*10;

	for(int i=0; i<N; i++)
	{
		amp[i] /= amp_counter;
		amp[i] /= N;
		amp[i] = log10((amp[i]))*10;
	}

	for(int i=0; i<N; i+=8)
	{
		fprintf(csv, "%d,%.3f,%f\n", i, (center_frequency+Index_to_frequency(20000000, N, i))/1000000.0f, amp[i]);
	}

	fftwf_destroy_plan(p);
	fftwf_free(in);
	fftwf_free(out);


	return 0;
}

