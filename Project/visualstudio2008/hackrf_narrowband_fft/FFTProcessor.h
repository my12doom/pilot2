#pragma once
#include "autolock.h"
#include "fftw/fftw3.h"

namespace NBFFT
{

enum fft_window_type
{
	fft_window_none,
	fft_window_hanning,
	fft_window_blackman_harris,
};

class FFTProcessor
{
public:
    FFTProcessor();
    ~FFTProcessor();
 
    int set_fft_size(int N);
    int set_alpha(float attack, float release);
    int set_window();

    int enable_dc_cancel();
    int enable_iq_correction();
    int enable_peak_search();

    int feed_data();
    int copy_bins();

protected:
    int N;
    fftwf_complex *fft_window;
    fftwf_complex * in;// = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
    fftwf_complex * out;// = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
    fftwf_plan p;
    _critical_section cs;
};

}
