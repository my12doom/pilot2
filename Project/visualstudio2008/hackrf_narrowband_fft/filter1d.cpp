#include "filter1d.h"
#include <string.h>
#include <stdio.h>

filter1d::filter1d()
{
    taps = NULL;
}

filter1d::filter1d(float *taps, int taps_count)
{
    this->taps = NULL;
    set_taps(taps, taps_count);
}

filter1d::~filter1d()
{
    if (taps)
        delete [] taps;
}

int filter1d::set_taps(float *taps, int taps_count)
{
    if (this->taps)
        delete [] this->taps;

    this->taps = new float[taps_count];
    memcpy(this->taps, taps, taps_count * sizeof(float));
    this->taps_count = taps_count;

    return 0;
}

int filter1d::apply(float *in, int intput_count, float *out /*= NULL */, int output_count /*= -1*/)
{
    if (output_count <= 0)
        output_count = intput_count;

    if (in == out)
        out = NULL;

    float *p = out;
    if (!out)
        p = new float[output_count];

    int half_tap = (taps_count - 1) / 2;
    if (intput_count == output_count)
    {
        for(int i=0; i<output_count; i++)
        {
            p[i] = 0;
            for(int j=0; j<taps_count; j++)
            {
                int idx = j - half_tap + i;
                if (idx < 0 || idx >= intput_count)
                    continue;
                p[i] += taps[taps_count-1-j] * in[idx];
            }
        }
    }
    
    if (!out)
    {
        memcpy(in, p, sizeof(float)*output_count);
        delete [] p;
    }

    return 0;
}
