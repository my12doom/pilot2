#pragma once
#include "fftw/fftw3.h"
#include "autolock.h"

namespace NBFFT
{
	class FFTBucket
	{
	public:
		FFTBucket();
		~FFTBucket();

		int resize(int N);
		int set_heatmap(int height, int range_high, int range_low);
		int set_heatmap_enable(bool enable);
		int set_vbw_hz(float attack, float release);

		fftwf_complex * lock_data(int start, int count);
		void unlock_data(const fftwf_complex *data);
		int copy_data(fftwf_complex *data_out, int start, int count);

		int feed_data(const fftwf_complex *data, int start, int count, float dt = 0);


	protected:
		int size;

		_critical_section cs;
	};
}
