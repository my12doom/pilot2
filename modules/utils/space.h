#pragma once

#include <stdio.h>

int space_init(bool erase = false);
int space_read(const void *key, int keysize, void *data, int num_to_read, int *num_read);
int space_write(const void *key, int keysize, const void *data, int num_to_write, int *num_written);
int space_delete(const void *key, int keysize);
int space_resort();
int space_available();
