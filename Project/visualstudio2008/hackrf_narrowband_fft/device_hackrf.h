#pragma once

int device_hackrf_init(int (*rx)(void *buf, int len, int type));
int device_hackrf_exit();