#pragma once

#include <stdint.h>
#include <time.h>
#include <Protocol/RFData.h>

extern bool storage_ready;
extern bool log_ready();

int sdcard_speed_test();
int format_sdcard();

int log_init();
int log(const void *packet, int size);
int log(const void *packet, uint8_t tag, int64_t timestamp);	// legacy support, 24byte fixed size packet
int log2(const void *packet, uint16_t tag, uint16_t size);
int log_write(const void *data, int size);		// write to file directly, do not use unless you know what you doing
int log_flush();
int log_set_time(time_t unix_time);				// set current time.


int open_firmware();
int write_firmware(const void*data, int count);
int close_and_check_firmware(int trucancate_size);

extern "C" int log_printf(const char*format, ...);