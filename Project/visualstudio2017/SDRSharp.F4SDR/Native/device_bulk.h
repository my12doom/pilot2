#pragma once


int device_bulk_init(int (*rx)(void *buf, int len, int type));
int device_bulk_exit();
int device_bulk_config();