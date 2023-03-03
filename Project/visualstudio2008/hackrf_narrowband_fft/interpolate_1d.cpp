#include "interpolate_1d.h"
#include <string.h>
#include <math.h>
#include <Windows.h>

int interpolate1d_linear(const float *in, float *out, int in_size, int out_size)
{
	float denom = 1.0f / (out_size-1);
	for(int i=0; i<out_size; i++)
	{
		float x = float(i) * (in_size-1) * denom;
		int l = floor(x);
		float alpha = x - l;
		out[i] = in[l] + alpha * (in[l+1]-in[l]);
	}
	return 0;
}

// 
// int interpolate1d_max_s(const float *in, float *out, int in_size, int out_size)
// {
// 	float denom = 1.0f / (out_size-1);
// 	for(int i=0; i<out_size; i++)
// 	{
// 		float x = float(i) * (in_size-1) * denom;
// 		int l = floor(x);
// 		out[i] = max(in[l-1], max(in[l], in[l+1]));
// 	}
// 	return 0;
// }

int interpolate1d_max(const float *in, float *out, int in_size, int out_size)
{
	if (out_size > in_size)
		return interpolate1d_linear(in, out, in_size, out_size);

	float scale = float(out_size-1) / (in_size-1);

	memset(out, 0, out_size * sizeof(float));

	for(int i=0; i<in_size; i++)
	{
		float x = float(i) * scale;
		int l = floor(x);
		out[l] = max(out[l], in[i]);
	}
	return 0;
}

int create_mipmap1d_max(const float *in, float *mipmap_out, int size)
{
	memcpy(mipmap_out, in, size*4);
	while (size > 0)
	{
		in = mipmap_out;
		mipmap_out += size;
		size /= 2;
		for(int i=0; i<size; i++)
			mipmap_out[i] = max(in[i*2], in[i*2+1]);
	}
	return 0;
}

int create_mipmap1d_minmax(const float *in, float *max_out, float *min_out, int size)
{
	memcpy(max_out, in, size*4);
	memcpy(min_out, in, size*4);

	const float *pmin;
	const float *pmax;
	while (size > 0)
	{
		pmax = max_out;
		pmin = min_out;
		max_out += size;
		min_out += size;
		size /= 2;
		for(int i=0; i<size; i++)
		{
			max_out[i] = max(pmax[i*2], pmax[i*2+1]);
			min_out[i] = min(pmin[i*2], pmin[i*2+1]);
		}
	}
	return 0;
}

int create_mipmap1d(const float *in, float *mipmap_out, int size)
{
	memcpy(mipmap_out, in, size*4);
	while (size > 0)
	{
		in = mipmap_out;
		mipmap_out += size;
		size /= 2;
		for(int i=0; i<size; i++)
			mipmap_out[i] = (in[i*2] + in[i*2+1]) * 0.5f;
	}
	return 0;
}


int interpolate1d_mipmap(const float *in, float *out, int in_size, int out_size, float *mipmap_cache/* = 0*/)
{
	float *mipmap = mipmap_cache;
	if (!mipmap)
	{
		mipmap = new float[in_size*2];
		create_mipmap1d(in, mipmap, in_size);
	}

	in = mipmap;
	while (in_size > out_size*2)
	{
		in += in_size;
		in_size/=2;
	}

	interpolate1d_linear(in, out, in_size, out_size);

	if (!mipmap_cache)
		delete mipmap;

	return 0;
}

int interpolate1d_mipmapmax(const float *in, float *out, int in_size, int out_size, float *mipmap_cache/* = 0*/)
{
	float *mipmap = mipmap_cache;
	if (!mipmap)
	{
		mipmap = new float[in_size*2];
		create_mipmap1d_max(in, mipmap, in_size);
	}

	in = mipmap;
	while (in_size > out_size*2)
	{
		in += in_size;
		in_size/=2;
	}

	interpolate1d_max(in, out, in_size, out_size);

	if (!mipmap_cache)
		delete mipmap;

	return 0;
}
