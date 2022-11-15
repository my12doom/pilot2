#pragma once

int create_mipmap1d(const float *in, float *mipmap_out, int size);
int create_mipmap1d_minmax(const float *in, float *max_out, float *min_out, int size);
int create_mipmap1d_max(const float *in, float *mipmap_out, int size);
int interpolate1d_linear(const float *in, float *out, int in_size, int out_size);
int interpolate1d_mipmap(const float *in, float *out, int in_size, int out_size, float *mipmap_cache = 0);
int interpolate1d_mipmapmax(const float *in, float *out, int in_size, int out_size, float *mipmap_cache = 0);
