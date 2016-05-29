#pragma once

#include <stdint.h>

typedef struct _feature_result
{
	int count;
	int x[2000];
	int y[2000];
	int v[2000];
} feature_result;

typedef struct _xy
{
	int x;
	int y;
}xy;

feature_result find_feature(uint8_t *image, int width, int height, int stride, uint8_t *image_out);
xy fit(uint8_t*image1, uint8_t*image2, int width, int height, int stride, int x, int y);
xy fit2(uint8_t*image1, uint8_t*image2, int width, int height, int stride, int x, int y);