#pragma once

#include <stdint.h>

int device_bulk_init(int (*rx)(void *buf, int len, int type));
int device_bulk_exit();
int device_bulk_config();
int device_bulk_tune(int64_t freq);
int device_bulk_gain(uint8_t gain);
int device_bulk_path(uint8_t path);
int device_bulk_control_io(bool tx, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, uint16_t length);
