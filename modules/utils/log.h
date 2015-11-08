#pragma once

#include <stdint.h>
#include <time.h>

extern bool log_ready;

int sdcard_speed_test();
int format_sdcard();

int log_init();
int log(const void *packet, int size);
int log(const void *packet, uint8_t tag, int64_t timestamp);	// legacy support, 24byte fixed size packet
int log2(const void *packet, uint16_t tag, uint16_t size);
int log_flush();
int log_set_time(time_t unix_time);				// set current time.


extern "C" int log_printf(const char*format, ...);