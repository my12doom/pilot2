#include "device_dummy.h"
#include <math.h>
#include <stdio.h>

static FILE * f = fopen("E:\\float.pcm", "wb");

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

		int tscale = GetTickCount() % 5000 < 2500 ? 32767 : 3276;
		float alpha = 1e-6;

        int count = sizeof(data)/sizeof(data[0]);
        float dp = 3000*2*PI/count;
        float dp2 = 9000*2*PI/count;
        for(int i=0; i<count; i+=2)
        {
			scale = tscale * alpha + scale * (1-alpha);
            phase[0] += dp;
            if (phase[0]>2*PI)
                phase[0] -= 2*PI;
            phase[1] += dp2;
            if (phase[1]>2*PI)
                phase[1] -= 2*PI;
			dataf[i+0] = (sinf(phase[0]) + sinf(phase[1]))/2;
			dataf[i+1] = 0;
			data[i+0] = floorf(dataf[i+0]*scale + tinymt32_generate_float(&tinymt)*0.5f);
            data[i+1] = 0;
        }

        if (cb)
            cb(data, sizeof(data)/sizeof(data[0]), sample_16bit);
    }
    return 0;
}

}
