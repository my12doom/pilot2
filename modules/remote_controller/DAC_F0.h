#pragma once
#include <HAL/STM32F0/F0GPIO.h>

int dac_run(const void *data, int data_point_count, int data_type, int repeat = 1);
int dac_config(STM32F0::F0GPIO *en, bool polary);
