#include <Windows.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "Fourier.h"
#include "hackrf.h"

#pragma comment(lib, "libhackrf.lib")
#pragma comment(lib, "libusb-1.0.lib")
#pragma comment(lib, "pthreadVSE2.lib")

hackrf_device* device = NULL;
#define N 8192

real_t real[N] = {0};
real_t image[N] = {0};

real_t real_out[N];
real_t image_out[N];

FILE * f = fopen("Z:\\log2.pcm", "wb");
FILE * csv = fopen("Z:\\spectrom.csv", "wb");

real_t amp[N] = {0};
int amp_counter = 0;
int rx_callback(hackrf_transfer* transfer)
{
	int8_t * buf = (int8_t *)transfer->buffer;
	int len = transfer->valid_length;

	for(int j=0; j<len/2/N; j++)
	{
		for(int i=0; i<N; i++)
		{
			real[i] = buf[i*2+1]/128.0f;
			image[i] = buf[i*2]/128.0f;
		}
 		fft_real_t(N, false, real, image, real_out, image_out);
		for(int i=0; i<N; i++)
		{
			real_t v = sqrt(real_out[i]*real_out[i]+image_out[i]*image_out[i]);
			amp[i] += v;
		}

		buf += N*2;
 		amp_counter++;
	}
 	fwrite(buf, 1, len, f);

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
	result |= hackrf_set_amp_enable(device, 0);
	result |= hackrf_set_sample_rate(device, 20000000);
	result |= hackrf_set_baseband_filter_bandwidth(device, 2.0e7);
	result |= hackrf_set_vga_gain(device, 0); // step: 2db
	result |= hackrf_set_lna_gain(device, 0); // step: 8db
	result |= hackrf_set_freq(device, 1410e6);

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
	fprintf(csv, "N,freq,amp\n");

	int res = StartHackRF();
	if (res != HACKRF_SUCCESS)
	{
		printf("%d:%s\n", res, hackrf_error_name((hackrf_error)res));
		return 0;
	}

	int t = GetTickCount();
	while(hackrf_is_streaming(device) == HACKRF_TRUE)
	{
		Sleep(10);

		if (GetTickCount() - t > 1000)
			hackrf_stop_rx(device);
	}

	hackrf_exit();

	float v = log10(127.0)*10;

	for(int i=0; i<N; i++)
	{
		amp[i] /= amp_counter;
		amp[i] /= N;
		amp[i] = log10(amp[i])*10;
	}

	for(int i=0; i<N; i++)
	{
		fprintf(csv, "%d,%.2f,%f\n", i, Index_to_frequency(20000000, N, i), amp[i]);
	}

	return 0;
}

