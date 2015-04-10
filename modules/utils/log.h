#pragma once

#include <stdint.h>

extern bool log_ready;
extern int log_pending;
extern int LOG_LEVEL;

int sdcard_speed_test();
int format_sdcard();

int log_init();
int log(const void *packet, int size);
int log(const void *packet, uint8_t tag, int64_t timestamp);	// legacy support, 24byte fixed size packet
int log_flush();