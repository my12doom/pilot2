#include "device_dummy.h"
#include <math.h>
#include <stdio.h>

namespace NBFFT
{
float PI = acos(-1.0);
int dummy_device::init(data_callback cb)
{
	tinymt32_init(&tinymt, 0);
    
    this->cb = cb;
    working = true;
	scale = 32767;
    memset(phase, 0, sizeof(phase));

    thread = CreateThread(NULL, NULL, entry, this, 0, 0);

    return 0;
}
int dummy_device::destroy()
{
    working = false;
    WaitForSingleObject(thread, INFINITE);

    return 0;
}

DWORD dummy_device::worker()
{
    while(working)
    {
        Sleep(1);

		int tscale = GetTickCount() % 5000 < 2500 ? 16380 : 16380;
		float noise_scale = 1.f;
		float alpha = 1e-6;

        int count = sizeof(data)/sizeof(data[0]);

		float f1 = 0.15;
		float f2 = 0.25;

		float m1 = 0.45;
		float m2 = 0.45;

        float dp = f1 * PI;
        float dp2 = f2 * PI;

		int nco_tbl[8] = 
		{
			1,1,
			1,-1,
			-1,-1,
			-1,1,
		};

		int nco_tbl2[8] = 
		{
			0,1,
			-1,0,
			0,-1,
			1,0,
		};
        for(int i=0; i<count; i+=2)
        {
			scale = tscale * alpha + scale * (1-alpha);
            phase[0] += dp;
            if (phase[0]>2*PI)
                phase[0] -= 2*PI;
			if (phase[0]<0)
				phase[0] += 2*PI;
            phase[1] += dp2;
            if (phase[1]>2*PI)
                phase[1] -= 2*PI;
			if (phase[1]<0)
				phase[1] += 2*PI;

			dataf[i+0] = (cosf(phase[0])*m1 + cosf(phase[1])*m2);
			dataf[i+1] = 0;//(sinf(phase[0])*m1 + sinf(phase[1])*m2);

			data[i+0] = dataf[i+0] * 32767 + (tinymt32_generate_float(&tinymt)-0.5f) * noise_scale;
			data[i+1] = dataf[i+1] * 32767 + (tinymt32_generate_float(&tinymt)-0.5f) * noise_scale;

// 			float ii = dataf[i+0] * nco_tbl[i%8] - dataf[i+1] * nco_tbl[(i+1)%8];
// 			float qq = dataf[i+1] * nco_tbl[i%8] + dataf[i+0] * nco_tbl[(i+1)%8];
// 
// 
// 			float ii2 = ii * nco_tbl2[i%8] - qq * nco_tbl2[(i+1)%8];
// 			float qq2 = qq * nco_tbl2[i%8] + ii * nco_tbl2[(i+1)%8];
// 
//  			dataf[i+0] = ii;
//  			dataf[i+1] = 0;//qq;
// 
// 			data[i+0] = floorf(dataf[i+0]*scale + (tinymt32_generate_float(&tinymt)-0.5f) * noise_scale);
//             data[i+1] = floorf(dataf[i+1]*scale + (tinymt32_generate_float(&tinymt)-0.5f) * noise_scale);

        }

        if (cb)
            cb(data, sizeof(data)/sizeof(data[0]));
    }
    return 0;
}

wav_device::wav_device(wchar_t *filename)
{
	f = _wfopen(filename, L"rb");
	thread = INVALID_HANDLE_VALUE;
}

int wav_device::init(data_callback cb)
{
	if (!f)
		return -1;

	fread(&hdr, 1, sizeof(hdr), f);
	quant = (sample_quant)(hdr.wex.wBitsPerSample >> 4);

	this->cb = cb;
	working = true;

	thread = CreateThread(NULL, NULL, entry, this, 0, 0);

	return 0;
}

int wav_device::destroy()
{
	working = false;
	if (thread != INVALID_HANDLE_VALUE)
	{
		WaitForSingleObject(thread, INFINITE);
		thread = INVALID_HANDLE_VALUE;
	}
	if (f)
		fclose(f);
	return 0;
}

DWORD wav_device::worker()
{
	if (!cb)
		return -1;

	size_t total_data = hdr.len + 8 - sizeof(WAV_HEADER);
	const size_t block_samples = 65536;
	uint32_t *data = new uint32_t[block_samples];
	size_t to_send = 0;
	int bit = hdr.wex.wBitsPerSample;
	quant = (sample_quant)(hdr.wex.wBitsPerSample >> 4);
	int byte_per_sample = hdr.wex.wBitsPerSample >> 3;

	int l = timeGetTime();
	while(working)
	{
		int cur = timeGetTime();
		int dtms = cur - l;
		l = cur;

		if (dtms <= 0)
		{
			Sleep(16);
			continue;
		}

		to_send += dtms * hdr.wex.nAvgBytesPerSec / 1000;

		while(to_send > block_samples * byte_per_sample)
		{
			if (feof(f))
			{
				fseek(f, sizeof(hdr), SEEK_SET);
			}

			fread(data, byte_per_sample, block_samples, f);

			if (hdr.wex.nChannels == 1)
			{
				// zero stuffing
				if (quant == sample_16bit)
				{
					int16_t *p = (int16_t*)data;
					for(int i=(block_samples-1)*2; i>=0; i-=2)
					{
						p[i] = p[i/2];
						p[i+1] = 0;
					}
				}

				cb(data, block_samples*2);
			}
			else
				cb(data, block_samples);

			to_send -= block_samples * byte_per_sample;
		}

		size_t pos = ftell(f) - sizeof(WAV_HEADER);
	}

	delete data;

	return 0;
}

DWORD wav_device::worker2()
{
	if (!cb)
		return -1;

	size_t total_data = hdr.len + 8 - sizeof(WAV_HEADER);
	const size_t block_size = 65536;
	uint32_t *data = new uint32_t[block_size];
	uint8_t *data_all = new uint8_t[total_data+block_size];
	int data_ptr = 0;

	fseek(f, sizeof(hdr), SEEK_SET);
	int n = fread(data_all, 1, total_data, f);

	size_t to_send = 0;
	int bit = hdr.wex.wBitsPerSample;
	sample_quant quant = (sample_quant)(hdr.wex.wBitsPerSample >> 4);
	int byte_per_sample = hdr.wex.wBitsPerSample >> 3;

	int l = timeGetTime();
	while(working)
	{
		int cur = timeGetTime();
		int dtms = cur - l;
		l = cur;

		if (dtms <= 0)
		{
			Sleep(16);
			continue;
		}

		to_send += dtms * hdr.wex.nAvgBytesPerSec / 1000;

		while(to_send > block_size * byte_per_sample)
		{
			if (data_ptr >= total_data)
				data_ptr = 0;
			memcpy(data, data_all + data_ptr, byte_per_sample*block_size);
			data_ptr += byte_per_sample*block_size;

			cb(data, block_size);

			to_send -= block_size * byte_per_sample;
		}

		size_t pos = ftell(f) - sizeof(WAV_HEADER);
	}

	delete data;
	delete data_all;

	return 0;
}

}
