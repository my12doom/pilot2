#include "device_hackrf.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include <windows.h>

namespace NBFFT
{

#define BLOCKS_PER_TRANSFER 16

int hackrf_device::rx_callback(hackrf_transfer* transfer)
{
	int8_t * buf = (int8_t *)transfer->buffer;
	int len = transfer->valid_length;

	uint8_t* ubuf;
	uint64_t frequency; /* in Hz */


	if (sweep)
	{
		for(buf; buf < (int8_t *)transfer->buffer+transfer->valid_length;) {
			ubuf = (uint8_t*) buf;
			if(ubuf[0] == 0x7F && ubuf[1] == 0x7F) {
				frequency = ((uint64_t)(ubuf[9]) << 56) | ((uint64_t)(ubuf[8]) << 48) | ((uint64_t)(ubuf[7]) << 40)
						| ((uint64_t)(ubuf[6]) << 32) | ((uint64_t)(ubuf[5]) << 24) | ((uint64_t)(ubuf[4]) << 16)
						| ((uint64_t)(ubuf[3]) << 8) | ubuf[2];
			} else {
				buf += BYTES_PER_BLOCK;
				continue;
			}


			static int l = GetTickCount();
			if (frequency == 2400000000)
			{
// 				printf("%llu,", frequency);

				printf("cycle time:%dms\n", GetTickCount()- l );
				l = GetTickCount();

			}
			if (sweep_cb)
			{
				sweep_info info = {frequency};
				sweep_cb(buf+10, BYTES_PER_BLOCK-10, info);
			}

			last_rx_time = GetTickCount();
			buf += BYTES_PER_BLOCK;
		}
	}
	else
	{
		if (cb)
			cb(buf, len);
		last_rx_time = GetTickCount();
	}

	return 0;
}

int hackrf_device::init(data_callback rx)
{
	destroy();

	cb = rx;
	_device = NULL;
	sweep = false;
	started = false;

	start_rx();

	// start watchdog
	watchdog_run = true;
	last_rx_time = GetTickCount();
	h_watchdog = CreateThread(NULL, NULL, watchdog_entry, this, NULL, NULL);

	return 0;
}

int hackrf_device::init(sweep_callback cb, sweep_config config)
{
	destroy();

	sweep_cb = cb;
	_device = NULL;
	sweep = true;
	started = false;
	_sweep_config = config;

	start_rx();

	// start watchdog
	watchdog_run = true;
	last_rx_time = GetTickCount();
	h_watchdog = CreateThread(NULL, NULL, watchdog_entry, this, NULL, NULL);

	return 0;
}

int hackrf_device::start_rx()
{
	int result = hackrf_init();
	if( result != HACKRF_SUCCESS ) {
		//printf("init_error");
		return result;
	}

	::hackrf_device *device = NULL;
	result = hackrf_open(&device);
	if( result != HACKRF_SUCCESS ) {
		//printf("device open_error");
		return result;
	}

	_device = device;

	result |= hackrf_set_amp_enable(device, 1);
	result |= hackrf_set_sample_rate(device, 20000000);
	result |= hackrf_set_baseband_filter_bandwidth(device, 15.0e6);
	result |= hackrf_set_vga_gain(device, 14); // step: 2db, 62db max
	result |= hackrf_set_lna_gain(device, 24); // step: 8db



	uint16_t v = 0;
	hackrf_max2837_read(device, 2, &v);

	int lpf_band = (v>>4)&0xf;
	char *lpf_tbl[] = {"1.75", "2.5", "3.5", "5.0", "5.5", "6.0", "7.0", "8.0", "9.0", "10.0", "12.0", "14.0", "15.0", "20.0", "24.0", "28.0"};
	printf("LPF:%sMhz for each I/Q, %.1fMhz total\n", lpf_tbl[lpf_band], atof(lpf_tbl[lpf_band])*2);

	result |= hackrf_start_rx(device, rx_callback_entry, this);

	// sweep
	if (sweep)
	{
		uint16_t freq_list[2] = {_sweep_config.freq_start/1e6, _sweep_config.freq_start/1e6 + _sweep_config.freq_step/1e6 * _sweep_config.points};	
		result |= hackrf_init_sweep(device, freq_list, 1, BYTES_PER_BLOCK, _sweep_config.freq_step, 0, LINEAR);
	}
	else
	{
		result |= hackrf_set_freq(device, center_frequency);
	}

	Sleep(1);
	return 0;
}
int hackrf_device::stop_rx()
{
	::hackrf_device *device = (::hackrf_device*)_device;
	if (device)
	{
		hackrf_stop_rx(device);
		hackrf_close(device);
		hackrf_exit();
	}

	started = false;
	_device = NULL;

	return 0;
}

int hackrf_device::destroy()
{
	// stop watchdog
	if (watchdog_run)
	{
		watchdog_run = false;
		WaitForSingleObject(h_watchdog, INFINITE);
	}

	stop_rx();

	return 0;
}

int hackrf_device::tune(int64_t hz)
{
	center_frequency = hz;
	::hackrf_device *device = (::hackrf_device*)_device;
	if (device)
		hackrf_set_freq(device, hz);
	return 0;
}

DWORD hackrf_device::watchdog()
{
	while(watchdog_run)
	{
		Sleep(16);
		if (GetTickCount() > last_rx_time + 1000L)
		{
			stop_rx();
			start_rx();
			last_rx_time = GetTickCount();
		}
	}

	return 0;
}

}
