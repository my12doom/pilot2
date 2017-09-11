#include "device_hackrf.h"
#include <stdint.h>
#include <stdio.h>
#include "hackrf.h"
#ifdef WIN32
#include <Windows.h>
#include <intrin.h>
#include <tmmintrin.h>
#else
#include <x86intrin.h>
#include <tmmintrin.h>
#endif

#pragma comment(lib, "libhackrf.lib")

hackrf_device* device = NULL;
static int (*rx)(void *buf, int len, int type) = NULL;
float center_frequency = 2370.0E6;

int rx_callback(hackrf_transfer* transfer)
{

	int8_t * buf = (int8_t *)transfer->buffer;
	int len = transfer->valid_length;

	if (rx)
		rx(buf, len, 0);

	// pulse slicer
	/*
	for(int i=0; i<len; i+=2)
	{
		int ampsq = buf[i] * buf[i] + buf[i+1] * buf[i+1];

		envelop = max(ampsq*1000, envelop);
		envelop = envelop * 99 / 100;

		power_lpf = -45;

		static int threshold_upper = pow(10, power_lpf/20) * 6 * 255;
		static int threshold_lower = pow(10, power_lpf/20) * 3 * 255;
		static bool b = false;

		if (!b)
		{
			threshold_upper = threshold_upper * threshold_upper * 1000;
			threshold_lower = threshold_lower * threshold_lower* 1000;
			b = true;
		}

		bool flag = false;
		if (!start && envelop > threshold_upper)
		{
			start = i+pos;
		}
		else if (start && envelop < threshold_lower)
		{
			int pkt_len = (i+pos-start)/2;
			start = 0;

			if (pkt_len > 24000 && pkt_len < 28000)
			{
				pkt_count ++;
				flag = true;
			}
		}

		buf[i] = start ? 50 : 0;
		if (flag)
			buf[i] = 100;
//  		buf[i+1] = sqrt(envelop/1000.0f);
	}

	pos += len;

	*/





// 
// 	float peak = 0;
// 	static float peak_lpf = 0;
// 	float alpha = 0.05;
// 
// 
// 	for(int i=0; i<len; i++)
// 	{
// 		float v = (buf[i] - DC)/128;
// 		peak = max(peak, v);
// 	}
// 	peak_lpf = peak * alpha + (1-alpha) * peak_lpf;
// // 	printf("\rpeak=%03.2f/%03.2f             ", log10(peak)*40, log10(peak_lpf)*40);
// 
// 	for(int j=0; j<len/2/N; j++)
// 	{
// 		for(int i=0; i<N; i++)
// 		{
// 			in[i][0] = buf[i*2+0]/128.0f;
// 			in[i][1] = buf[i*2+1]/128.0f;
// 		}
// 		fftwf_execute(p);
// 		for(int i=1; i<N; i++)
// 		{
// 			float v = sqrt(out[i][0]*out[i][0] + out[i][1]*out[i][1]);
// 			amp[i] += v;
// 		}
// 
// 		buf += N*2;
//  		amp_counter++;
// 	}
// 
// 
// 	printf("\ravg power:%.2f %.3f, %d pkts   ", power_lpf, pow(10, power_lpf/20) * 6 * 255, pkt_count);
	return 0;
}


int8_t *data;
int data_count = 0;

int tx_callback(hackrf_transfer* transfer)
{
	int8_t * buf = (int8_t *)transfer->buffer;
	int len = transfer->valid_length;

	memset(buf, 0, len);
	memcpy(buf, data, min(data_count, len));

	static int c = 0;
	printf("\r%d", c++);

	return 0;
}


int start_rx()
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


	result |= hackrf_set_amp_enable(device, 1);
	result |= hackrf_set_sample_rate(device, 20000000);
	result |= hackrf_set_baseband_filter_bandwidth(device, 25e6);
	result |= hackrf_set_vga_gain(device, 16); // step: 2db
	result |= hackrf_set_lna_gain(device, 24); // step: 8db
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

int tx()
{
	// 	FILE * f = fopen("Z:\\wifi_6mbps_1frame.pcm", "rb");
	// 	if (!f)
	// 	{
	// 		printf("failed opening file\n");
	// 		return 0;
	// 	}
	// 	fseek(f, 0, SEEK_END);
	// 	int file_size = ftell(f);
	// 	fseek(f, 0, SEEK_SET);
	// 
	// 	int sample_count = file_size/4;
	// 	short * data16 = new short[file_size/2];
	// 	data = new int8_t[sample_count*2];
	// 	fread(data16, 1, file_size, f);
	// 	fclose(f);
	// 
	// 	data_count = sample_count*2;
	// 	for(int i=0; i<sample_count; i++)
	// 	{
	// 		data[i*2+0] = data16[i*2+0] / 256;
	// 		data[i*2+1] = data16[i*2+1] / 256;
	// 	}
	// 
	// 	f = fopen("Z:\\test_8bit.pcm", "wb");
	// 	fwrite(data, 2, sample_count, f);
	// 	fclose(f);




	FILE * f = fopen("Z:\\out_8bit.pcm", "rb");
	if (!f)
	{
		printf("failed opening file\n");
		return 0;
	}
	fseek(f, 0, SEEK_END);
	int file_size = ftell(f);
	fseek(f, 0, SEEK_SET);
	int sample_count = file_size/2;
	data_count = file_size;
	data = new int8_t[data_count];
	fread(data, 1, file_size, f);
	fclose(f);



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
	result |= hackrf_set_txvga_gain(device, 47);
	result |= hackrf_set_freq(device, center_frequency);

	uint16_t v = 0;
	LARGE_INTEGER l1,l2,freq;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&l1);
	hackrf_max2837_read(device, 2, &v);
	QueryPerformanceCounter(&l2);
	float s = (float)(l2.QuadPart-l1.QuadPart) / (freq.QuadPart);

	result |= hackrf_start_tx(device, tx_callback, NULL);

	while(hackrf_is_streaming(device) == HACKRF_TRUE && GetKeyState(VK_END) >=0)
	{
		Sleep(10);

		// 		if (GetTickCount() - t > 1000)
		// 			hackrf_stop_rx(device);
	}

	hackrf_stop_tx(device);
	hackrf_exit();

	exit(0);
}

int device_hackrf_init(int (*rx)(void *buf, int len, int type))
{
	::rx = rx;
	return start_rx();
}
int device_hackrf_exit()
{
	hackrf_stop_rx(device);
	hackrf_exit();

	return 0;
}