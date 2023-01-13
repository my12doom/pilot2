#include <Windows.h>
#include <WindowsX.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "fftw/fftw3.h"
#include "sse_mathfun.h"
#include "autolock.h"
#include "resource.h"
#include <float.h>
#include <CommCtrl.h>

#include "device_hackrf.h"
#include "device_bulk.h"
#include "device_dummy.h"
#include "device_fx3.h"

#include "filter1d.h"
#include "interpolate_1d.h"

#include <intrin.h>
#include <tmmintrin.h>
#include "FFTProcessor.h"
#include "fifo.h"
#include "sinc.h"

#include "DSOwindow.h"

#include <Protocol/common.h>

#include <math/matrix.h>

#pragma comment(lib, "pthreadVSE2.lib")
#pragma comment(lib, "fftw/libfftw3f-3.lib")
#pragma comment(lib, "fftw/libfftw3-3.lib")
using namespace NBFFT;

// configuration
char *rbw_list[] = {"1Khz", "10Khz", "100Khz", "1Mhz", "custom", "FFT bins"};
char *device_list[] = {"hackrf", "dummy", "bulk", "wav file", "FX3"};
const int minN = 32;
const int maxN = 4096*1024;
const int minHitmapN = 512;
const int maxHitmapN = 4096;

// structure/enum declaration
enum rbw_type
{
	rbw_none,
	rbw_gaussion,
	rbw_box,
};


// functions
int set_rbw_filter_bins(rbw_type type, float bw3db_bins, float dynamic_range_db);
int set_rbw_filter_hz(rbw_type type, float bw3db_hz, float dynamic_range_db);
int set_window();
int int8tofloat(float *dst, const int8_t*src, int count, float scale=1/128.0f);
int int16tofloat(float *dst, const int16_t*src, int count, float scale=1/32768.0f);
float total(const float *data, int count);
void total_complex(const float *data, float *out, int count);
int set_fft_size(int N);
int open_device(const char *name, void *extra = NULL);
int record_open(const char *filename = NULL);
int record_close();

// variables
//static const double PI = acos(-1.0);
float range_low = -130;
float range_high = 0;
filter1d rbw_filter;
rbw_type g_rbw_type = rbw_gaussion;
sample_quant quant_type = sample_16bit;
int byte_per_sample;
device *d = NULL;
sample_type device_sample_type;

float sample_rate = 20e6;
float bw_per_bin;
float rbw = 10000;

int N = 64*1024;
bool auto_fft_size = true;
fftwf_complex * fft_in = NULL;//
fftwf_complex * fft_out = NULL;
fftwf_plan plan_complex = NULL;
fftwf_plan plan_real = NULL;
float ampsq[maxN];
float peaksq_amp = 0;
float ms_amp = 0;	// ms = mean square / average power
float fft_window_real[maxN];
float fft_window_complex[maxN*2];
float fft_window_correction = 0;
float DC[2] = {0};
_critical_section cs_amp;
_critical_section cs_rbw;
_critical_section cs_fft;
_critical_section cs_device;
_critical_section cs_outfile;
HWND hDlg;
HWND slider;
HWND slider2;
float phase_imba = 0.0f;
float gain_imba = 1.0f;

FILE * f = NULL;
int canvas_width = N;
int canvas_height;
RGBQUAD color_tbl[256];

const int amp_resolution = 512;
uint8_t __declspec(align(16)) hitmap8[amp_resolution+1][maxHitmapN*4] = {0};
int * hitmap = (int*)_aligned_malloc(maxHitmapN*(amp_resolution+1)*4, 16);//
float __declspec(align(16)) hitmap_merged[amp_resolution+1][maxHitmapN] = {0};
uint8_t __declspec(align(16)) hitmap_merged8[amp_resolution+1][maxHitmapN] = {0};
int hitmap_count = 0;
float hitmap_time = 0;
bool update_hitmap = false;
bool exiting = false;
int marker_N = -1;

HANDLE hSA_thread;
HANDLE hdraw_thread;
bool enable_SA = false;


MagicRingBuffer SA_ringbuf(4*65536*2);
_critical_section cs_SA_ringbuf;

__forceinline __m128i _mm_mulhi_epu8(__m128i a, __m128i b)
{
	__m128i m0 = _mm_setzero_si128();
	__m128i mhia = _mm_unpackhi_epi8(m0, a);
	__m128i mloa = _mm_unpacklo_epi8(m0, a);
	__m128i mhib = _mm_unpackhi_epi8(m0, b);
	__m128i mlob = _mm_unpacklo_epi8(m0, b);

	mhia = _mm_mulhi_epu16(mhia, mhib);
	mloa = _mm_mulhi_epu16(mloa, mlob);

	mhia = _mm_srli_epi16(mhia, 8);
	mloa = _mm_srli_epi16(mloa, 8);

	return _mm_packus_epi16(mloa, mhia);
}

int merge_hitmap(float dt)
{
	float alpha_attack = 0.99f;
	float alpha_release = 0.1f;
	float alpha_attack_m1 = 1- alpha_attack;
	float alpha_release_m1 = 1 - alpha_release;

	if (N > maxHitmapN)
		return -1;

	__m128 mattack_a = _mm_set1_ps(alpha_attack);
	__m128 mrelease_a = _mm_set1_ps(alpha_release);
	__m128 m1attack_a = _mm_sub_ps(_mm_set1_ps(1), mattack_a);
	__m128 m1release_a = _mm_sub_ps(_mm_set1_ps(1), mrelease_a);
	__m128 mfactor = _mm_set1_ps(N*2.5f/1024);
	__m128 m0 = _mm_set1_ps(0.0f);
	__m128i m0i = _mm_setzero_si128();
	__m128 m255 = _mm_set1_ps(255.0f);
	__m128 mff;
	_mm_storeu_si128((__m128i*)&mff, _mm_set1_epi8(0xff));

	{
	uint8_t a[16] = {10,20,30,40,50,60};
	uint8_t b[16] = {10,20,30,40,50,60,70,80};
	uint8_t ref[16];
	for(int i=0; i<16; i++)
		ref[i] = (a[i]*b[i])>>8;


	__m128i ma = _mm_loadu_si128((__m128i*)a);
	__m128i mb = _mm_loadu_si128((__m128i*)b);

	__m128i c = _mm_unpacklo_epi8(ma, m0i);
	c = _mm_unpacklo_epi16(c, m0i);
	ma = _mm_mulhi_epu8(ma, mb);
	_mm_storeu_si128((__m128i*)a, ma);
	_mm_storeu_si128((__m128i*)a, ma);

	}


// 	// apply 3*3 kernel & convert to int
// 	for(int y=0; y<amp_resolution; y++)
// 	{
// 		memset(hitmap + y*maxHitmapN, 0, N * sizeof(hitmap[0]));
// 	}
// 
// 	int N1 = N-1;
// 	for(int y=1; y<amp_resolution; y++)
// 	{
// 		for(int x=0; x<N; x+=16)
// 		{
// 			for(int j=1; j<=16; j++)
// 			{
// 				int v = hitmap8[y][x+j];
// 				int v3 = v * 3;
// 				hitmap[y*maxHitmapN + ((x + j))] += 9*v;
// 
// 				hitmap[(y+1)*maxHitmapN + ((x + j + 0))] += v3;
// 				hitmap[(y+0)*maxHitmapN + ((x + j + 1))] += v3;
// 				hitmap[(y+0)*maxHitmapN + ((x + j - 1))] += v3;
// 				hitmap[(y-1)*maxHitmapN + ((x + j + 0))] += v3;
// 
// 				hitmap[(y+1)*maxHitmapN + ((x + j + 1))] += v;
// 				hitmap[(y-1)*maxHitmapN + ((x + j - 1))] += v;
// 				hitmap[(y+1)*maxHitmapN + ((x + j - 1))] += v;
// 				hitmap[(y-1)*maxHitmapN + ((x + j - 1))] += v;
// 			}
// 		}
// 	}

	__m128i mscale = _mm_set1_epi32(4+4*3+9);

	for(int y=0; y<amp_resolution; y++)
	{
		for(int x=0; x<N; x+=16)
		{
			__m128i v[4];
			__m128i m8 = _mm_loadu_si128((__m128i*)&hitmap8[y][x]);

			for(int j=0; j<4; j++)
			{
				__m128i m1i = _mm_unpacklo_epi8(m8, m0i);
				m1i = _mm_unpacklo_epi16(m1i, m0i);
				m1i = _mm_mullo_epi32(m1i, mscale);
				m8 = _mm_srli_si128(m8, 4);

				//__m128i m1i = _mm_loadu_si128((__m128i*)&hitmap[y*maxHitmapN + x+4*j]);
				__m128 m1;
				m1 = _mm_cvtepi32_ps(m1i);
				m1 = _mm_mul_ps(m1, mfactor);

				// attack & release
				__m128 pre = _mm_load_ps(&hitmap_merged[y][x] + 4*j);
				__m128 mattack = _mm_mul_ps(m1, mattack_a);
				mattack = _mm_add_ps(mattack, _mm_mul_ps(pre, m1attack_a));
				__m128 mrelease = _mm_mul_ps(m1, mrelease_a);
				mrelease = _mm_add_ps(mrelease, _mm_mul_ps(pre, m1release_a));

				__m128 m2 = _mm_cmpge_ps(m1, pre);
				__m128 m2n = _mm_xor_ps(m2, mff);
				mattack = _mm_and_ps(mattack, m2);
				mrelease = _mm_and_ps(mrelease, m2n);

				m1 = _mm_or_ps(mattack, mrelease);

				// constrain to [0,255]
				m1 = _mm_max_ps(m1, m0);
				m1 = _mm_min_ps(m1, m255);

				_mm_store_ps(&hitmap_merged[y][x] + 4*j, m1);

				v[j] = _mm_cvtps_epi32(m1);
			}

			v[0] = _mm_packus_epi32(v[0], v[1]);
			v[2] = _mm_packus_epi32(v[2], v[3]);
			v[0] = _mm_packus_epi16(v[0], v[2]);
			_mm_store_si128((__m128i*)&hitmap_merged8[y][x], v[0]);
		}
	}

	return 0;
}

int process_fft_block(float *ampsq_copy, void *buf, bool update_hitmap)
{
	float dt = N/sample_rate;
	float alpha_attack =dt / (dt + 1.0f/(25*PI * 1));
	float alpha_release = dt / (dt + 1.0f/(25*PI * 1));

	__m128 m10 = _mm_set1_ps(10/log(10.0f));
	__m128 mscale = _mm_set1_ps((amp_resolution-1)/(range_high - range_low));
	__m128 mhigh = _mm_set1_ps(range_high);
	__m128 mlow = _mm_set1_ps(range_low);

	__m128 mattack_a = _mm_set1_ps(alpha_attack);
	__m128 mrelease_a = _mm_set1_ps(alpha_release);
	__m128 m1attack_a = _mm_sub_ps(_mm_set1_ps(1), mattack_a);
	__m128 m1release_a = _mm_sub_ps(_mm_set1_ps(1), mrelease_a);
	__m128 mN = _mm_set1_ps((device_sample_type == real_sample ? 4.0 : 1.0)/N/N);		// 4.0 = 2*2 for real sample
	__m128 mff;
	_mm_storeu_si128((__m128i*)&mff, _mm_set1_epi8(0xff));

	// convert/copy data
	int multi = device_sample_type == complex_sample ? 2 : 1;
	if (sample_8bit == quant_type)
		int8tofloat((float*)fft_in, (int8_t*)buf, N*multi);
	else if (sample_16bit == quant_type)
		int16tofloat((float*)fft_in, (int16_t*)buf, N*multi);
	else if (sample_float == quant_type)
		memcpy(fft_in, buf, N*sizeof(float)*multi);

	// find DC offset
	__m128 mdc = {0};
	float DC_N[2] = {0};
	total_complex((float*)fft_in, DC_N, N);
	float alpha_DC = dt / (dt + 1.0f/(5.5f*PI * 1));
	DC[0] = DC[0] * (1-alpha_DC) + DC_N[0] * alpha_DC / N;
	DC[1] = DC[1] * (1-alpha_DC) + DC_N[1] * alpha_DC / N;

	if (complex_sample == device_sample_type)
	{
		mdc = _mm_set_ps(DC[1], DC[0], DC[1], DC[0]);

		// find peak and rms
		__m128 mpeak = _mm_set1_ps(0);		
		__m128 mavg = _mm_set1_ps(0);		
		for(int i=0; i<N*2; i+=8)
		{
			float *p = (float *)fft_in;
			__m128 m1 = _mm_loadu_ps(p+i);
			//m1 = _mm_sub_ps(m1, mdc);
			m1 = _mm_mul_ps(m1, m1);
			__m128 m2 = _mm_loadu_ps(p+i+4);
			//m2 = _mm_sub_ps(m2, mdc);
			m2 = _mm_mul_ps(m2, m2);
			m1 = _mm_hadd_ps(m1, m2);
			mpeak = _mm_max_ps(mpeak, m1);
			mavg = _mm_add_ps(mavg, m1);
		}
		float alpha_avg = pow(1-alpha_release, N/1000);

		float *pk = (float*)&mpeak;
		peaksq_amp *= alpha_avg;
		peaksq_amp = max(max(pk[0], pk[1]), max(pk[2], pk[3]));
		float *pavg = (float*)&mavg;

		ms_amp *= alpha_avg;
		ms_amp += (1-alpha_avg) * (pavg[0]+pavg[1]+pavg[2]+pavg[3]) / N;
	}

	// apply DC offset correction and fft window
	if (complex_sample == device_sample_type)
	{
		for(int i=0; i<N*2; i+=4)
		{
			__m128 ma = _mm_loadu_ps(((float *)fft_in)+i);
			__m128 mb = _mm_loadu_ps(fft_window_complex+i);
	 		ma = _mm_sub_ps(ma, mdc);
			ma = _mm_mul_ps(ma, mb);
			_mm_storeu_ps(((float *)fft_in)+i, ma);
		}
	}
	else
	{
		__m128 mdc = _mm_set_ps1((DC[0] + DC[1])/2);

		for(int i=0; i<N*2; i+=4)
		{
			__m128 ma = _mm_loadu_ps(((float *)fft_in)+i);
			__m128 mb = _mm_loadu_ps(fft_window_real+i);
 			ma = _mm_sub_ps(ma, mdc);
			ma = _mm_mul_ps(ma, mb);
			_mm_storeu_ps(((float *)fft_in)+i, ma);
		}
	}


	if (device_sample_type == complex_sample)
		fftwf_execute(plan_complex);
	else
		fftwf_execute(plan_real);


	__m128 _m1 = _mm_set1_ps(0.0f);
	__m128 _mamp = _mm_set1_ps(amp_resolution-2);
	int N1 = N-1;
	for(int j=0; j<N; j+=4)
	{
		__m128 m1 = _mm_loadu_ps((float*)&fft_out[(j+N/2)&N1]);		// reorder from 0 ~ fs/2, -fs/2 ~ 0 to -fs/2 ~ fs/2
		__m128 m2 = _mm_loadu_ps((float*)&fft_out[(j+N/2+2)&N1]);

		m1 = _mm_mul_ps(m1, m1);
		m2 = _mm_mul_ps(m2, m2);
		m1 = _mm_hadd_ps(m1, m2);
		m1 = _mm_mul_ps(m1, mN);

		// hitman calculate
		if (N <= maxHitmapN && update_hitmap)
		{
			int tmp[4];
			__m128 logpwr = log_ps(m1);
			logpwr = _mm_mul_ps(logpwr, m10);
			logpwr = _mm_max_ps(logpwr, mlow);
			logpwr = _mm_min_ps(logpwr, mhigh);
			logpwr = _mm_sub_ps(logpwr, mlow);
			logpwr = _mm_mul_ps(logpwr, mscale);
			logpwr = _mm_max_ps(logpwr, _m1);
			logpwr = _mm_min_ps(logpwr, _mamp);
			_mm_storeu_si128((__m128i*)tmp, _mm_cvtps_epi32(logpwr));

			for(int i=0; i<4; i++)
				hitmap8[tmp[i]][i+j] ++ ;
		}

		// attack & release
		__m128 pre = _mm_loadu_ps(ampsq_copy+j);
		__m128 mattack = _mm_mul_ps(m1, mattack_a);
		mattack = _mm_add_ps(mattack, _mm_mul_ps(pre, m1attack_a));
		__m128 mrelease = _mm_mul_ps(m1, mrelease_a);
		mrelease = _mm_add_ps(mrelease, _mm_mul_ps(pre, m1release_a));

		m2 = _mm_cmpge_ps(m1, pre);
		__m128 m2n = _mm_xor_ps(m2, mff);
		mattack = _mm_and_ps(mattack, m2);
		mrelease = _mm_and_ps(mrelease, m2n);

		m1 = _mm_or_ps(mattack, mrelease);

		_mm_storeu_ps(ampsq_copy+j, m1);
	}
	
	// merge hitmap
	if (N <= maxHitmapN)
	{
		hitmap_count ++;
		hitmap_time += dt;
		if (hitmap_time > 16e-3 || hitmap_count == 255)
		{
			merge_hitmap(dt);

			for(int i=0; i<amp_resolution; i++)
				memset(hitmap8[i], 0, N*sizeof(hitmap8[i][0]));
			hitmap_time = 0;
			hitmap_count = 0;
		}
	}

	return 0;
}


int data_handler(void *buf, int len)
{
	{
		_autolock lck(&cs_outfile);
		if (f)
		{
			if (quant_type == sample_8bit)
			{
				// convert signed 8bit to unsigned
				static uint8_t *buf2 = new uint8_t[maxN*4];
				for(int i=0; i<len*byte_per_sample; i++)
					buf2[i] = ((int8_t*)buf)[i] + 128;
				fwrite(buf2, 1, len*byte_per_sample, f);
				//delete buf2;
			}
			else
			{
				fwrite(buf, 1, len*byte_per_sample, f);
			}
		}
	}
	
	int result = 0;

	{
		_autolock lck(&cs_SA_ringbuf);
		if (enable_SA)
 			result += SA_ringbuf.push_back((uint8_t*)buf, len*byte_per_sample);
	}

	{
		_autolock lck(&cs_DSO_ringbuf);
		result += DSO_ringbuf.push_back((uint8_t*)buf, len*byte_per_sample);
	}

	return result > 0 ? 0 : -1;
}

DWORD WINAPI SA_thread(LPVOID p)
{
	float * ampsq_copy = new float[maxN];
	uint8_t *buf = new uint8_t[4*maxN*2];
	while (!exiting)
	{
		int block_size = byte_per_sample*2*N;

		if (!enable_SA)
		{
			Sleep(100);
			continue;
		}


		cs_SA_ringbuf.enter();
		if (SA_ringbuf.used_space() < block_size)
		{
			cs_SA_ringbuf.leave();
	
			Sleep(1);
			continue;
		}

		SA_ringbuf.pop_front(buf, block_size);
		cs_SA_ringbuf.leave();

		cs_amp.enter();
		memcpy(ampsq_copy, ampsq, N*sizeof(float));
		cs_amp.leave();

		cs_fft.enter();
		process_fft_block(ampsq_copy, buf, update_hitmap);
		cs_fft.leave();

		cs_amp.enter();
		memcpy(ampsq, ampsq_copy, N*sizeof(float));
		cs_amp.leave();
	}

	delete ampsq_copy;
	delete buf;

	return 0;
}




RGBQUAD graph[1024*1024];
RGBQUAD waterfall[1024*1024] = {0};

int draw_line(RGBQUAD*canvas, int x1, int y1, int x2, int y2, RGBQUAD color)	// draw line in pixel space
{
	int dx = x2 - x1;
	int dy = y2 - y1;
	int ux = ((dx > 0) << 1) - 1;
	int uy = ((dy > 0) << 1) - 1;
	int x = x1, y = y1, eps;

	eps = 0;dx = abs(dx); dy = abs(dy); 
	if (dx > dy) 
	{
		for (x = x1; x != x2; x += ux)
		{
			if (y>=0 && y<canvas_height && x>=0 && x<canvas_width)
				canvas[y*canvas_width+x] = color;
			eps += dy;
			if ((eps << 1) >= dx)
			{
				y += uy;
				eps -= dx;
			}
		}
	}
	else
	{
		for (y = y1; y != y2; y += uy)
		{
			if (y>=0 && y<canvas_height && x>=0 && x<canvas_width)
				canvas[y*canvas_width+x] = color;
			eps += dx;
			if ((eps << 1) >= dy)
			{
				x += ux;
				eps -= dy;
			}
		}
	}

	return 0;
}

float amp[maxN];
float amp_avg[maxN*2];
float amp_max[maxN*2];
float amp_min[maxN*2];
// float amp_tmp[maxN];
#define perf dts[counter] = timeGetTime() - t; lines[counter++] = __LINE__;

int last_draw_time = timeGetTime();

#include <libyuv/scale.h>

bool init = false;
RECT rect;
int width;
int height;
HDC hdc;
HDC memDC;
HBITMAP bitmap;
HGDIOBJ obj;
RECT rect_waterfall;
int width_wf;
int height_wf;
HDC hdc_wf;
HDC memDC_wf;
HBITMAP bitmap_wf;
HGDIOBJ obj_wf;
int draw_init(HWND hwnd_graph, HWND hwnd_waterfall)
{
	if (init)
		return 0;
	init = true;

	GetClientRect(hwnd_graph, &rect);
	hdc = GetDC(hwnd_graph);
	memDC = CreateCompatibleDC(hdc);
	bitmap = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
	obj = SelectObject(memDC, bitmap);

	width = rect.right - rect.left;
	height = rect.bottom - rect.top;

	GetClientRect(hwnd_waterfall, &rect_waterfall);
	width_wf = rect_waterfall.right - rect_waterfall.left;
	height_wf = rect_waterfall.bottom - rect_waterfall.top;
	hdc_wf = GetDC(hwnd_waterfall);
	memDC_wf = CreateCompatibleDC(hdc_wf);
	bitmap_wf = CreateCompatibleBitmap(hdc_wf, width_wf, height_wf);
	obj_wf = SelectObject(memDC_wf, bitmap_wf);

	return 0;
}

float N2freq(int n)
{
	if (device_sample_type == complex_sample)
		return int64_t(n-N/2)* sample_rate /N*1e-6;
	else
		return int64_t(n-N/2)* sample_rate /N*1e-6;
}

int draw_deinit(HWND hwnd_graph, HWND hwnd_waterfall)
{
	init = false;


	DeleteObject(obj);
	DeleteObject(bitmap);
	DeleteDC(memDC);
	ReleaseDC(hwnd_graph, hdc);

	DeleteObject(obj_wf);
	DeleteObject(bitmap_wf);
	DeleteDC(memDC_wf);
	ReleaseDC(hwnd_waterfall, hdc_wf);
	
	return 0;
}

void draw_text(char *text, int center_x, int center_y, RGBQUAD font_color)
{
	if (text[0] == NULL)
		return;

	HDC hdc = GetDC(NULL);
	HDC hdcBmp = CreateCompatibleDC(hdc);

	LOGFONTW lf={0};
	lf.lfHeight = -16;
	lf.lfCharSet = GB2312_CHARSET;
	lf.lfOutPrecision =  OUT_STROKE_PRECIS;
	lf.lfClipPrecision = CLIP_STROKE_PRECIS;
	lf.lfQuality = DEFAULT_QUALITY;
	lf.lfPitchAndFamily = VARIABLE_PITCH;
	lf.lfWeight = FW_BOLD*3;
	lstrcpynW(lf.lfFaceName, L"ºÚÌå", 32);

	static HFONT m_font = 0;
	if (!m_font)
		m_font = CreateFontIndirectW(&lf);

	HFONT hOldFont = (HFONT) SelectObject(hdcBmp, m_font);

	RECT rect = {0,0,1920, 1080};
	DrawTextA(hdcBmp, text, strlen(text), &rect, DT_CENTER | DT_CALCRECT | DT_WORDBREAK | DT_NOFULLWIDTHCHARBREAK | DT_EDITCONTROL);
	int width_pixel = rect.right - rect.left;
	int height_pixel = rect.bottom - rect.top;

	HBITMAP hbm = CreateCompatibleBitmap(hdc, width_pixel, height_pixel);

	HBITMAP hbmOld = (HBITMAP)SelectObject(hdcBmp, hbm);

	RECT rcText;
	SetRect(&rcText, 0, 0, width_pixel, height_pixel);
	SetBkColor(hdcBmp, RGB(0, 0, 0));					// Pure black background
	SetTextColor(hdcBmp, RGB(255, 255, 255));			// white text for alpha

	DrawTextA(hdcBmp, text, strlen(text), &rect, DT_CENTER);

	BYTE * data = (BYTE *) malloc(width_pixel * height_pixel * 4);
	GetBitmapBits(hbm, width_pixel * height_pixel * 4, data);


	RGBQUAD *data_rgb = (RGBQUAD*)data;
	unsigned char color_r = font_color.rgbRed;
	unsigned char color_g = font_color.rgbGreen;
	unsigned char color_b = font_color.rgbBlue;
	DWORD color = (color_r<<16) | (color_g <<8) | (color_b);// reverse big endian

	for(int y=0; y<height_pixel; y++)
	for(int x=0; x<width_pixel; x++)
	{

		int yy = y - height_pixel/2 + center_y;
		int xx = x - width_pixel/2 + center_x;

		if (yy >=0 && yy < canvas_height && xx>=0 && xx < canvas_width)
		{

			unsigned char alpha = data_rgb[y*width_pixel+x].rgbRed;
			unsigned char alpha1 = 255 - alpha;

			RGBQUAD &p = graph[yy*canvas_width + xx];
			p.rgbRed = (p.rgbRed * alpha1 + color_r * alpha) >> 8;
			p.rgbGreen = (p.rgbGreen * alpha1 + color_g * alpha) >> 8;
			p.rgbBlue = (p.rgbBlue * alpha1 + color_b * alpha) >> 8;
		}
	}

	delete data;
	DeleteObject(SelectObject(hdcBmp, hbmOld));
	SelectObject(hdc, hOldFont);
	DeleteObject(hbm);
	DeleteDC(hdcBmp);
	ReleaseDC(NULL, hdc);
}

int mN;
float zoom_left = 0.0f;
float zoom_right = 1.0f;
int draw(HWND hwnd_graph, HWND hwnd_waterfall)
{
	draw_init(hwnd_graph, hwnd_waterfall);

	int t = timeGetTime();
	int fps = 1000 / (t - last_draw_time);
	last_draw_time = t;
	int dts[100];
	int lines[100];
	int counter = 0;

	perf

	// drawing region
	canvas_width = width;
	canvas_height = height;
	memset(graph, 0x20, sizeof(graph));
	RGBQUAD red = {0,0,255,255};
	RGBQUAD blue = {255,0,0,255};
	RGBQUAD light_red = {180,180,255,255};
	RGBQUAD yellow = {0,255,255,255};

	// hitmap
	update_hitmap = BST_CHECKED == SendMessage(GetDlgItem(hDlg, IDC_HEATMAP), BM_GETCHECK, 0, 0);

	if (N <= maxHitmapN && update_hitmap)
	{
		static uint8_t *plane = new uint8_t[maxHitmapN * maxHitmapN];

		if (complex_sample == device_sample_type)
			libyuv::ScalePlane(&hitmap_merged8[0][0], maxHitmapN, N, amp_resolution, plane, 2048, width, height, libyuv::kFilterBilinear);
		else
			libyuv::ScalePlane(&hitmap_merged8[0][N/2], maxHitmapN, N/2, amp_resolution, plane, 2048, width, height, libyuv::kFilterBilinear);


		for(int y=0; y<height; y++)
		{
			RGBQUAD *p = &graph[y*width];

			for(int x=0; x<width; x++)
				p[x] = color_tbl[plane[(height-1 - y)*2048+x]];
		}
	}

	perf

	// axies
	for(int i=range_low; i<=range_high; i+=10)
	{
		int y = height * (range_high - i) / (range_high-range_low);
		for(int j=0; j<width; j++)
			graph[y*width+j] = light_red;
	}

	perf
	float amp_pixel[3][2048];
	uint8_t waterfall_line[2048];
	cs_amp.enter();
	memcpy(amp, ampsq, N*sizeof(float));
	cs_amp.leave();
	perf

	float zoom_width = zoom_right - zoom_left;


	if (g_rbw_type != rbw_none && rbw > 100)
		rbw_filter.apply(amp, N);
	perf

	if (device_sample_type == complex_sample)
	{
		float *zoomed_amp = amp + int(zoom_left*N);
		int bin_count = zoom_width*N;

		create_mipmap1d_minmax(zoomed_amp, amp_max, amp_min, bin_count);
		create_mipmap1d(zoomed_amp, amp_avg, bin_count);
		interpolate1d_mipmap(zoomed_amp, amp_pixel[0], bin_count, width, amp_avg);
		interpolate1d_mipmap(zoomed_amp, amp_pixel[1], bin_count, width, amp_min);
		interpolate1d_mipmapmax(zoomed_amp, amp_pixel[2], bin_count, width, amp_max);
	}
	else
	{
		float *zoomed_amp = amp + N/2 + int(zoom_left*N/2);
		int bin_count = zoom_width*N/2;

		create_mipmap1d_minmax(zoomed_amp, amp_max, amp_min, bin_count);
		create_mipmap1d(zoomed_amp, amp_avg, bin_count);
		interpolate1d_mipmap(zoomed_amp, amp_pixel[0], bin_count, width, amp_avg);
		interpolate1d_mipmap(zoomed_amp, amp_pixel[1], bin_count, width, amp_min);
		interpolate1d_mipmapmax(zoomed_amp, amp_pixel[2], bin_count, width, amp_max);
	}

	mN = marker_N * width / N;
	float tmp[3];
	for(int i=0; i<3; i++)
		tmp[i] = log10(amp_pixel[i][mN]);

	// draw lines
	float last_min = -100;
	float last_max = -100;
	for(int i=0; i<width; i++)
	{
		float dbavg = (amp_pixel[0][i]>0) ? log10(amp_pixel[0][i]) * 10 : range_low;
		float dbmin = (amp_pixel[1][i]>0) ? log10(amp_pixel[1][i]) * 10 : range_low;
		float dbmax = (amp_pixel[2][i]>0) ? log10(amp_pixel[2][i]) * 10 : range_low;

		dbavg = limit(dbavg, range_low, range_high);
		dbmin = limit(dbmin, range_low, range_high);
		dbmax = limit(dbmax, range_low, range_high);

		float vavg = (height-1) * (range_high - dbavg) / (range_high-range_low);
		float vmin = (height-1) * (range_high - dbmin) / (range_high-range_low);
		float vmax = (height-1) * (range_high - dbmax) / (range_high-range_low);

		waterfall_line[i] = 255 * (dbavg - range_low) / (range_high-range_low);

		if (i>0)
		{
			draw_line(graph, i, vmax, i, vmin, yellow);
			if (last_max < vmin)
				draw_line(graph, i-1, last_max, i, vmin, yellow);
			if (last_min > vmax)
				draw_line(graph, i-1, last_min, i, vmax, yellow);

		}

		last_min = vmin;
		last_max = vmax;
	}

	// draw marker
	if (marker_N >= 0)
	{
		int i = (device_sample_type == complex_sample) ? (marker_N * width / N) : ((marker_N-N/2) * width * 2 / N);
		float dbmax = (amp[marker_N]>0) ? log10(amp[marker_N]) * 10 : range_low;
		float vmax = (height-1) * (range_high - dbmax) / (range_high-range_low);
		int size = 3;
		for(int j=-size; j<=size; j++)
			draw_line(graph, i-j, vmax-size, i-j, vmax+size, red);

		char text[100];
		sprintf(text, "%.4fMhz\n%.2fdbFS", N2freq(marker_N), dbmax);
		draw_text(text, i, vmax-25, red);
	}

	perf
	// overall amplitude
	float peakamp_db = min(max(range_low,log10(peaksq_amp)*10), range_high);
 	draw_line(graph, 0, height, 0, height * (range_high - peakamp_db) / (range_high-range_low), red);

	// transfer & release
	SetBitmapBits(bitmap, rect.right * rect.bottom * 4, graph);
	BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);


	// draw waterfall
	memmove(waterfall + width, waterfall, sizeof(RGBQUAD)*width*(height-1));
	for(int i=0; i<width; i++)
		waterfall[i] = color_tbl[waterfall_line[i]];

	SetBitmapBits(bitmap_wf, width_wf * height_wf * 4, waterfall);
	BitBlt(hdc_wf, 0, 0, width_wf, height_wf, memDC_wf, 0, 0, SRCCOPY);


	perf
// 	printf("%dfps,", fps);
// 
// 	printf("\r");
// 	for(int i=0; i<counter; i++)
// 	{
// 		printf("%d@%d,", dts[i], lines[i]);
// 	}
// 	printf("      ");
	return 0;
}

void update_marker()
{
	HWND graph = GetDlgItem(hDlg, IDC_GRAPH);
	RECT rect;
	GetClientRect(graph, &rect);
	int width = rect.right - rect.left;
	int height = rect.bottom - rect.top;

	POINT mouse;
	GetCursorPos(&mouse);
	ScreenToClient(graph, &mouse);

	char tmp[100] = {0};

	// mouse marker
	if(mouse.x >=0 && mouse.x < width-1 && mouse.y >= 0 && mouse.y < height - 1)
	{
		int selN = device_sample_type == complex_sample ? (mouse.x * N / width) : (N/2 + mouse.x * N/2 / width);
		if(GetKeyState(VK_LBUTTON) < 0)
			marker_N = selN;

		sprintf(tmp, "mouse:%.4fMhz, %.2fdbfs", N2freq(selN), log10(ampsq[selN])*10);

	}

	if (marker_N >=0 && marker_N < N)
	{
		char tmp2[100];
		sprintf(tmp2, "\nmark1:%.4fMhz, %.2fdbfs", N2freq(marker_N), log10(ampsq[marker_N])*10);
		strcat(tmp, tmp2);

		static int last_l = 0;
		static int last_r = 0;
		if (GetKeyState(VK_LEFT) != last_l)
		{
			last_l = GetKeyState(VK_LEFT);
			if (last_l < 0)
				marker_N = limit(marker_N - 1, 0, N);
		}
		if (GetKeyState(VK_RIGHT) != last_r)
		{
			last_r = GetKeyState(VK_RIGHT);
			if (last_r < 0)
				marker_N = limit(marker_N + 1, 0, N);
		}
	}
	SetDlgItemTextA(hDlg, IDC_MARKER, tmp);

	// peak search

	static bool last_v = false;
	if (GetKeyState(VK_HOME) < 0 || GetKeyState(VK_PRIOR) < 0 || GetKeyState(VK_NEXT) < 0 || GetKeyState(VK_CONTROL) < 0)
	{
		const int peak_count = 10;
		int min_distance = 2 * N / 1024;
		int peak_pos[peak_count] = {0};
		float peak_amp[peak_count] = {0};
		static int peak_sel = 0;

		for(int k=0; k<peak_count; k++)
		{
			float a = 0;
			int tmp = 0;
			for(int i=(device_sample_type == complex_sample ? 0 : N/2); i<N; i++)
			{
				bool _near = false;
				for(int j=0; j<k; j++)
				{
					if (abs(i-peak_pos[j]) < min_distance)
					{
						_near = true;
						break;
					}
				}
				if (_near)
					continue;

				if (ampsq[i] > a)
				{
					a = ampsq[i];
					tmp = i;
				}
			}
			peak_pos[k] = tmp;
			peak_amp[k] = log10(a)*10;
//  			peak_amp[k] = a;
		}

		if (GetKeyState(VK_HOME) < 0)
		{
			peak_sel = 0;
			marker_N = peak_pos[peak_sel];
		}

		if (last_v == false)
		{
		if (GetKeyState(VK_PRIOR) < 0)
		{
			peak_sel = limit(peak_sel-1, 0, peak_count-1);
			marker_N = peak_pos[peak_sel];
		}

		if (GetKeyState(VK_NEXT) < 0)
		{
			peak_sel = limit(peak_sel+1, 0, peak_count-1);
			marker_N = peak_pos[peak_sel];
		}

		// peak tracking
		if (GetKeyState(VK_CONTROL) < 0)
			marker_N = peak_pos[peak_sel];
		}

		last_v = true;
	}
	else
	{
		last_v = false;
	}
		

}



VOID CALLBACK timer_cb(HWND dlg, UINT n, UINT_PTR ptr, DWORD what)
{
	if (!enable_SA)
	{
		Sleep(100);
		return;
	}

	draw(GetDlgItem(hDlg, IDC_GRAPH), GetDlgItem(hDlg, IDC_WATERFALL));
	update_marker();
	//set_rbw_filter(rbw_gaussion, SendMessage(slider, TBM_GETPOS, 0, 0)+1, 100);
	float peakamp_db = log10(peaksq_amp)*10;	// -3: 2channel
	float rms_db = log10(ms_amp)*10 + fft_window_correction;
	float dc_amp = DC[0]*DC[0] + DC[1]*DC[1];
	float dc_db = log10(dc_amp)*10;
	char tmp[100];
	sprintf(tmp, "%.2f/%.2f peak/rms dbFS, %.2fLSB, 10db/div, DC:%.2fdb", peakamp_db, rms_db, sqrt(ms_amp)*32767, dc_db);
 	SetWindowTextA(hDlg, tmp);

}

DWORD WINAPI draw_thread(LPVOID p)
{
	int l = GetTickCount();
	while(!exiting)
	{
		int t = GetTickCount();
		if (t - l > 10)
		{
			timer_cb(hDlg, 0, NULL, 0);
			l = t;
		}
		else
		{
			Sleep(1);
		}
	}

	return 0;
}

int init_dialog(HWND hDlg)
{
//	window_start = GetTickCount();
// 	SetTimer(NULL, 0, 15, timer_cb);
	hdraw_thread = CreateThread(NULL, NULL, draw_thread, NULL, 0, 0);
	hSA_thread = CreateThread(NULL, NULL, SA_thread, NULL, 0, 0);

	slider = GetDlgItem(hDlg, IDC_SLIDER1);
	slider2 = GetDlgItem(hDlg, IDC_SLIDER2);
	SendMessage(slider, TBM_SETRANGEMAX, (WPARAM)TRUE, (LPARAM)100);
	SendMessage(slider, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)0);
	SendMessage(slider2, TBM_SETRANGEMAX, (WPARAM)TRUE, (LPARAM)1000);
	SendMessage(slider2, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)500);

	// devices
	for(int i=0; i<sizeof(device_list)/sizeof(device_list[0]); i++)
		SendMessageA(GetDlgItem(hDlg, ID_CB_DEVICES), CB_ADDSTRING, 0, (LPARAM)device_list[i]);
	SendMessageA(GetDlgItem(hDlg, ID_CB_DEVICES), CB_SETCURSEL, 0, 0);


	HWND cb = GetDlgItem(hDlg, ID_CB_RBW);
	SendMessageA(cb, CB_RESETCONTENT , 0, 0);
	for(int i=0; i<sizeof(rbw_list)/sizeof(rbw_list[0]); i++)
		SendMessageA(cb, CB_ADDSTRING, 0, (LPARAM)rbw_list[i]);
	SendMessageA(cb, CB_SETCURSEL, 1, 0);
	ShowWindow(GetDlgItem(hDlg, IDC_RBW), SW_HIDE);
	ShowWindow(GetDlgItem(hDlg, IDC_RBW_HZ), SW_HIDE);

	HWND fft = GetDlgItem(hDlg, ID_CB_FFTSIZE);
	SendMessageA(fft, CB_ADDSTRING, 0, (LPARAM)"Auto");
	for(int n = minN, i = 0; n<=maxN; n*=2, i++)
	{
		char tmp[20];
		sprintf(tmp, "%d", n);
		SendMessageA(fft, CB_ADDSTRING, 0, (LPARAM)tmp);
		if (n == N)
			SendMessageA(fft, CB_SETCURSEL, i, 0);
	}
	SendMessageA(fft, CB_SETCURSEL, 0, 0);

	SetDlgItemTextA(hDlg, IDC_FREQ, "2500000000");
	SetDlgItemTextA(hDlg, IDC_GAIN, "10");

	return 0;
}

int clearup()
{
	exit_dso();
	exiting = true;
	WaitForSingleObject(hSA_thread, INFINITE);
	WaitForSingleObject(hdraw_thread, INFINITE);

	if (d)
	{
		d->destroy();
		delete d;
	}

	fftwf_destroy_plan(plan_complex);
	fftwf_free(fft_in);
	fftwf_free(fft_out);

	return 0;
}

int update_rbw(HWND hDlg)
{
	HWND cb = GetDlgItem(hDlg, ID_CB_RBW);
	char tmp[100] = {0};
	GetWindowTextA(cb, tmp, sizeof(tmp)-1);

	if (strstr(tmp, "custom"))
	{
		ShowWindow(GetDlgItem(hDlg, IDC_RBW), SW_SHOW);
		ShowWindow(GetDlgItem(hDlg, IDC_RBW_HZ), SW_SHOW);

		sprintf(tmp, "%d", int(rbw));
		SetDlgItemTextA(hDlg, IDC_RBW, tmp);

		return 0;
	}
	else
	{
		ShowWindow(GetDlgItem(hDlg, IDC_RBW), SW_HIDE);
		ShowWindow(GetDlgItem(hDlg, IDC_RBW_HZ), SW_HIDE);		
	}
	
	int n = atoi(tmp);
	if (strstr(tmp, "Khz"))
		n *= 1000;
	if (strstr(tmp, "Mhz"))
		n *= 1000000;

	if (strstr(tmp, "FFT bins"))
		n = 0;
	
	int o = set_rbw_filter_hz(rbw_gaussion, n, -range_low);

	return o;
}

int update_fft_size(HWND cb)
{
	char tmp[100] = {0};
	GetWindowTextA(cb, tmp, sizeof(tmp)-1);

	return set_fft_size(atoi(tmp));
}

int apply_gain(HWND hDlg)
{
	char tmp[100] = {0};
	GetDlgItemTextA(hDlg, IDC_GAIN, tmp, sizeof(tmp)-1);
	int gain = atoi(tmp);
	if (BST_CHECKED == SendMessage(GetDlgItem(hDlg, IDC_GAIN_LNA), BM_GETCHECK, 0, 0))
		gain |= 0x80;

	cs_device.enter();
	if(d)
		d->set_gains((uint8_t*)&gain);
	cs_device.leave();

	printf("Gain:%d | %x\n", gain&0x7f, gain & 0x80);

	return 0;
}

INT_PTR CALLBACK main_window_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	switch( msg ) 
	{
	case WM_MOUSEWHEEL:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
			POINT p = {xPos, yPos};
			HWND pwnd = WindowFromPoint(p);
			if (GetDlgCtrlID(pwnd) == IDC_FREQ)
			{				
				char tmp[100] = {0};
				GetDlgItemTextA(hDlg, IDC_FREQ, tmp, sizeof(tmp)-1);
				int64_t hz = _strtoui64(tmp, NULL, 10);

				hz += zDelta * 1e6 / WHEEL_DELTA;
				if (hz<10000000)
					hz = 10000000;

				if (hz>0)
				{
					cs_device.enter();
					if(d)
						d->tune(hz);
					cs_device.leave();

					sprintf(tmp, "%lld", int64_t(hz));
					SetDlgItemTextA(hDlg, IDC_FREQ, tmp);
				}
			}

			else if (GetDlgCtrlID(pwnd) == IDC_GAIN)
			{
				char tmp[100] = {0};
				GetDlgItemTextA(hDlg, IDC_GAIN, tmp, sizeof(tmp)-1);
				int gain = atoi(tmp);

				gain += zDelta / WHEEL_DELTA;
				
				gain = gain<0 ? 0 : (gain>127?127:gain);
				sprintf(tmp, "%d", gain);
				SetDlgItemTextA(hDlg, IDC_GAIN, tmp);

				apply_gain(hDlg);
			}
		}
		break;
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			int event = HIWORD(wParam);

			if (id == IDC_RECORD)
			{
				cs_outfile.enter();
				if (f)
				{
					record_close();
					SetDlgItemTextA(hDlg, IDC_RECORD, "Record");
				}
				else
				{
					if (record_open() == 0)
						SetDlgItemTextA(hDlg, IDC_RECORD, "Stop Recording");
				}
				cs_outfile.leave();
			}

			if (id == IDC_DSO)
				show_dso_window(NULL, NULL);

			if (id == IDC_GAIN_LNA)
				apply_gain(hDlg);

			if (id == IDC_OPEN)
			{
				char tmp[100] = {0};
				GetDlgItemTextA(hDlg, ID_CB_DEVICES, tmp, sizeof(tmp)-1);
				open_device(tmp);
			}

			if (id == ID_CB_RBW && HIWORD(wParam) == CBN_SELCHANGE)
			{
				update_rbw(hDlg);
			}

			if (id == ID_CB_FFTSIZE && HIWORD(wParam) == CBN_SELCHANGE)
			{
				update_fft_size(GetDlgItem(hDlg, ID_CB_FFTSIZE));
				update_rbw(hDlg);
			}

			if (id == IDC_RBW && HIWORD(wParam) == EN_CHANGE)
			{
				char tmp[100] = {0};
				GetDlgItemTextA(hDlg, IDC_RBW, tmp, sizeof(tmp)-1);
				float hz = atof(tmp);
				if (hz>0)
					set_rbw_filter_hz(rbw_gaussion, atoi(tmp), -range_low);
			}

			if (id == IDC_FREQ)
			{
				char tmp[100] = {0};
				GetDlgItemTextA(hDlg, IDC_FREQ, tmp, sizeof(tmp)-1);
				int64_t hz = _strtoui64(tmp, NULL, 10);

				if (hz < 10e6)
				{
					hz = 10e6;
					SetDlgItemTextA(hDlg, IDC_FREQ, "10000000");
				}

				if (hz>0)
				{
					cs_device.enter();
					if(d)
						d->tune(hz);
					cs_device.leave();
				}
			}

			if (id == IDC_CONFIG)
			{
				cs_device.enter();
				if(d)
					d->config();
				cs_device.leave();
			}

			enable_SA = BST_CHECKED == SendMessage(GetDlgItem(hDlg, IDC_SA), BM_GETCHECK, 0, 0);
		}
		break;

	case WM_INITDIALOG:
		::hDlg = hDlg;
		init_dialog(hDlg);
		update_rbw(hDlg);
		break;
	case WM_HSCROLL:
		phase_imba = (SendMessage(slider, TBM_GETPOS, 0, 0)-500)*2*PI/180/500;
		gain_imba = (SendMessage(slider2, TBM_GETPOS, 0, 0)-500)*0.05f/500 + 1.0f;
		break;

	case WM_CLOSE:
		EndDialog(hDlg, 0);
		break;

	default:
		return FALSE;
	}

	return TRUE; // Handled message
}

int set_rbw_filter_bins(rbw_type type, float bw3db_bins, float dynamic_range_db)
{
	// RBW gaussion filter
	if (type == rbw_gaussion)
	{
		bw3db_bins /= 2;

		float dynamic_range = pow(10, dynamic_range_db / 10);
		float a = log(0.5) / (- bw3db_bins * bw3db_bins);		// at x=bw3db_bins, exp(-a*x^2) = 0.5
		float d = exp(-a * bw3db_bins * bw3db_bins);			// == -3db (0.5)

		float x2sq = -(dynamic_range_db/10)*log(10.0)/(-a) + (bw3db_bins*bw3db_bins);
		float x2 = sqrt(x2sq);
		float d2 = exp(-a * x2 * x2);							// == 0.5 * 10^(dynamic_range_db/10)

		int taps_half = floor(x2) + 1;
		float *rbw_taps = new float[taps_half * 2 + 1];
		float total = 0;
		for(int i=0; i<=taps_half; i++)
		{
			rbw_taps[i+taps_half] = exp(-a*i*i);
			rbw_taps[taps_half-i] = rbw_taps[i+taps_half];

			total += rbw_taps[i+taps_half] * 2;
		}
		rbw_filter.set_taps_fft(rbw_taps, taps_half * 2 + 1, N);
		//rbw_filter.set_taps(rbw_taps, taps_half * 2 + 1);

		printf("%d taps\n", taps_half * 2 + 1);

		delete rbw_taps;
	}

	g_rbw_type = type;
	return 0;
}

int set_rbw_filter_hz(rbw_type type, float bw3db_hz, float dynamic_range_db)
{
	if (auto_fft_size && bw3db_hz > 0)
	{
		float minimun_N = (sample_rate / bw3db_hz) * 5;		// 5x over sampling
		int newN = powf(2, ceil(log(minimun_N)/log(2.0f)));
		if (N != newN)
		{
			printf("setting fft size to %d\n", newN);
			set_fft_size(newN);
		}
	}

	rbw = bw3db_hz;
	bw_per_bin = sample_rate / N;

	char tmp[200];
	sprintf(tmp, "effective RBW: %.0fhz, FFT%d", rbw?rbw:bw_per_bin, N);
	SetDlgItemTextA(hDlg, IDC_RBW_REAL, tmp);

	if (bw3db_hz < 1)
	{
		g_rbw_type = rbw_none;
		rbw = bw_per_bin;
		printf("RBW filter disabled, fft rbw=%.1f hz\n", rbw);

		return 0;
	}

	if (bw_per_bin > bw3db_hz)
		printf("too narrow rbw requested: %.1f, but minimum is %.1f hz\n", bw3db_hz, bw_per_bin);	

	return set_rbw_filter_bins(type, bw3db_hz/bw_per_bin, 120);
}

#define _mm_cmpge_epu8(a, b) _mm_cmpeq_epi8(_mm_max_epu8(a, b), a)
#define _mm_cmpge_epu16(a, b) _mm_cmpeq_epi16(_mm_max_epu16(a, b), a)

int int8tofloat(float *dst, const int8_t*src, int count, float scale/*=1/128.0f*/)
{
	__m128 mscale = _mm_set1_ps(scale);
	__m128i m80 = _mm_set1_epi8(0x80);
	__m128i m8000 = _mm_set1_epi16(0x8000);
	for(int i=0; i<count; i+=16)
	{
		__m128i m1 = _mm_loadu_si128((__m128i*)(src+i));
		__m128i mh = _mm_cmpge_epu8(m1, m80);
		__m128i m2 = _mm_unpackhi_epi8(m1, mh);
		m1 = _mm_unpacklo_epi8(m1, mh);

		mh = _mm_cmpge_epu16(m2, m8000);
		__m128i m4 = _mm_unpackhi_epi16(m2, mh);
		__m128i m3 = _mm_unpacklo_epi16(m2, mh);

		mh = _mm_cmpge_epu16(m1, m8000);
		m2 = _mm_unpackhi_epi16(m1, mh);
		m1 = _mm_unpacklo_epi16(m1, mh);

		__m128 m1f = _mm_mul_ps(_mm_cvtepi32_ps(m1), mscale);
		__m128 m2f = _mm_mul_ps(_mm_cvtepi32_ps(m2), mscale);
		__m128 m3f = _mm_mul_ps(_mm_cvtepi32_ps(m3), mscale);
		__m128 m4f = _mm_mul_ps(_mm_cvtepi32_ps(m4), mscale);

		_mm_storeu_ps(dst+i+0, m1f);
		_mm_storeu_ps(dst+i+4, m2f);
		_mm_storeu_ps(dst+i+8, m3f);
		_mm_storeu_ps(dst+i+12, m4f);
	}

	return 0;
}

int int16tofloat(float *dst, const int16_t*src, int count, float scale/* = 1/32768.0f*/)
{
	__m128 mscale = _mm_set1_ps(scale);
	__m128 m_offset = _mm_set1_ps(0.5f/32767);
	__m128i m8000 = _mm_set1_epi16(0x8000);
	for(int i=0; i<count; i+=8)
	{
		__m128i m1 = _mm_loadu_si128((__m128i*)(src+i));

		__m128i mh = _mm_cmpge_epu16(m1, m8000);
		__m128i m2 = _mm_unpackhi_epi16(m1, mh);
		m1 = _mm_unpacklo_epi16(m1, mh);

		__m128 m1f = _mm_mul_ps(_mm_cvtepi32_ps(m1), mscale);
		__m128 m2f = _mm_mul_ps(_mm_cvtepi32_ps(m2), mscale);

		m1f = _mm_add_ps(m1f, m_offset);
		m2f = _mm_add_ps(m2f, m_offset);

		_mm_storeu_ps(dst+i+0, m1f);
		_mm_storeu_ps(dst+i+4, m2f);
	}

	return 0;
}

float total(float *data, int count)
{
	__m128 m = _mm_set1_ps(0);
	for(int i=0; i<count; i+=4)
	{
		__m128 d = _mm_loadu_ps(data+i);
		m = _mm_add_ps(m, d);		
	}
	float *p = (float*)&m;
	return p[0] + p[1] + p[2] + p[3];
}

void total_complex(const float *data, float *out, int count)
{
	__m128 m = _mm_set1_ps(0);
	for(int i=0; i<count*2; i+=4)
	{
		__m128 d = _mm_loadu_ps(data+i);
		m = _mm_add_ps(m, d);		
	}
	float *p = (float*)&m;
	out[0] = p[0] + p[2];
	out[1] = p[1] + p[3];
}

int set_fft_size(int _N)
{

	if (_N > maxN)
		return -1;

	cs_fft.enter();
	if (_N == 0)
	{
		auto_fft_size = true;
		set_rbw_filter_hz(g_rbw_type, rbw, -range_low);
		cs_fft.leave();

		return 0;
	}
// 	else
// 	{
// 		auto_fft_size = false;
// 	}

	marker_N = -1;

	N = _N;
	if (plan_complex)
		fftwf_destroy_plan(plan_complex);
	if (plan_real)
		fftwf_destroy_plan(plan_real);
	if (fft_in)
		fftwf_free(fft_in);
	if (fft_out)
		fftwf_free(fft_out);

	fft_in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
	fft_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
	plan_complex = fftwf_plan_dft_1d(N, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
	plan_real = fftwf_plan_dft_r2c_1d(N, (float*)fft_in, fft_out, FFTW_ESTIMATE);

	// flat-top window function
// 	float a0 = 0.21557895;
// 	float a1 = 0.41663158;
// 	float a2 = 0.277263158;
// 	float a3 = 0.083578947;
// 	float a4 = 0.006947368;

	// blackman-harris window function
	float a0 = 0.35875;
	float a1 = 0.48829;
	float a2 = 0.14128;
	float a3 = 0.01168;
	float a4 = 0;
	for(int i=0; i<N; i++)
	{
		fft_window_real[i] = (a0 - a1*cos(2*PI*i/N) + a2*cos(4*PI*i/N) - a3*cos(6*PI*i/N) + a4*cos(8*PI*i/N))/a0;	// /a0: normalized
		//fft_window[i] = 1;
		fft_window_complex[i*2] = fft_window_complex[i*2+1] = fft_window_real[i];
	}

	fft_window_correction = -3.0194;


	// small initial value
	for(int i=0; i<N; i++)
	{
		ampsq[i] = 1e-15;
	}
	cs_fft.leave();

	//set_rbw_filter_hz(g_rbw_type, rbw, -range_low);
	_autolock lck(&cs_SA_ringbuf);	
	SA_ringbuf.alloc(max(512*1024, N*4*4));


	return 0;
}

int open_device(const char *name, void *extra /*= NULL*/)
{
	cs_device.enter();
	if (d)
	{
		d->destroy();
		delete d;
		d = NULL;
	}
	cs_device.leave();

	device * p = NULL;

	if (strcmp(name, "hackrf") == 0)
		p = new NBFFT::hackrf_device;

	if (strcmp(name, "bulk") == 0)
		p = new NBFFT::bulk_device;

	if (strcmp(name, "FX3") == 0)
		p = new NBFFT::fx3_device;

	if (strcmp(name, "dummy") == 0)
		p = new NBFFT::dummy_device;

	if (strcmp(name, "wav file") == 0)
	{
// 		p = new NBFFT::wav_device(L"C:\\Users\\mo\\Desktop\\repo\\pilot2\\Project\\visualstudio2017\\SDRSharp.F4SDR\\bin\\Debug\\SDRSharp_20220622_082212Z_2500000000Hz_IQ.wav");
		p = new NBFFT::wav_device(L"D:\\temp\\tst.wav");
	}

	if (!p)
		return -1;

	cs_device.enter();
	d = p;
	d->init(data_handler);

	sample_rate = d->get_sample_rate();
	device_sample_type = d->get_sample_type();
	quant_type = d->get_sample_quant();
	byte_per_sample = (sample_8bit == quant_type) ? 1 : ((sample_16bit==quant_type) ? 2 : 4);
	range_high = 0;
	range_low = -d->dynamic_range_db();
	cs_device.leave();

	set_rbw_filter_hz(g_rbw_type, rbw, -range_low);

	return 0;
}



int record_open(const char *filename/* = NULL*/)
{
	if (!filename)
		filename = "D:\\temp\\log.wav";

	cs_outfile.enter();
	if (!f)
		record_close();

	f = fopen(filename, "wb");

	if (f)
	{
		WAV_HEADER hdr = {};
		fwrite(&hdr, 1, sizeof(hdr), f);

	}
	cs_outfile.leave();

	return f?0:-1;
}

int record_close()
{
	cs_outfile.enter();
	if (f)
	{
		// update header
		size_t data_size = ftell(f) - sizeof(WAV_HEADER);
		int bytes_per_point = 1 << quant_type;
		int channel_count = d->get_sample_type() == real_sample ? 1 : 2;

		WAVEFORMATEX wex = 	{
			WAVE_FORMAT_PCM,
			channel_count,
			d->get_sample_rate(),
			bytes_per_point * d->get_sample_rate() * channel_count,
			bytes_per_point * channel_count,
			bytes_per_point * 8,
			0x6461	// "data"
		};

		WAV_HEADER hdr;

		memcpy(hdr.riff, "RIFF", 4);
		hdr.len = data_size + sizeof(WAV_HEADER) - 8;
		memcpy(hdr.cWavFmt, "WAVEfmt ", 8);
		hdr.dwHdrLen = 16;
		hdr.wex = wex;
		hdr.wex.cbSize = 0x6164; // da
		hdr.symbol_ta = 0x6174; //ta
		hdr.dwDataLen = (DWORD)data_size;


		fseek(f, 0, SEEK_SET);
		fwrite(&hdr, 1, sizeof(hdr), f);
		fclose(f);
		f = NULL;
	}
	cs_outfile.leave();

	return 0;
}

void simple_decimator(const char *input, const char *output)
{
	FILE * f = fopen(input, "rb");
	if (!f)
		return;

	FILE * o = fopen(output, "wb");
	if (!o)
	{
		fclose(f);
		return;
	}

	int16_t tmp[2000*2];
	fread(tmp, 1, 100, f);
	fwrite(output, 1, 100, o);
	while (fread(tmp, 1, sizeof(tmp), f) == sizeof(tmp))
	{
		for(int i=0; i<200; i++)
		{
			int o[2] = {0};
			for(int j=0; j<10; j++)
			{
				o[0] += tmp[i*20+j];
				o[1] += tmp[i*20+j+1];
			}
			tmp[i*2] = o[0]/10;
			tmp[i*2+1] = o[1]/10;
		}

		fwrite(tmp, 1, sizeof(tmp)/10, o);
	}

	fclose(f);
	fclose(o);
}

const int n = 4096*1024;
int16_t data[n][2];
double damp[n] = {0};
int phase_noise2()
{
	int sample_rate = 125e6;
	int bw = 2e6;
	float freq_resolution = (float)sample_rate / n;
	float freq_res_dbhz = log10(freq_resolution) * 10;
	int window_spreading = 4;
	float signal_bw = max(1e4, freq_resolution*(window_spreading*2+2));		// max of 10khz or fft window spreading


	set_fft_size(n);
	memset(damp, 0, sizeof(damp));
	float window_conv = 0;
	for(int i=0; i<n; i++)
		window_conv += fft_window_real[i];

// 	char file_old[] = "D:\\temp\\log.wav";
	char file_in[] = "D:\\temp\\100cry.wav";
// 	simple_decimator(file_old, file_in);
	char file_out[MAX_PATH];
	sprintf(file_out, "%s.csv", file_in);

	FILE * f = fopen(file_in, "rb");
	fseek(f, 144, SEEK_SET);

	int avg_count = 0;
	int skip_count = 0;

	int skip_table[] = {-1};

	while((fread(data, 1, 4*n, f) == 4*n) && avg_count < 30)
	{

// 		if (avg_count == 1)
// 			continue;

		bool skip = false;
		for(int i=0; i<sizeof(skip_table)/sizeof(skip_table[0]); i++)
		{
			if (skip_table[i] == avg_count)
			{
				skip = true;
				skip_count ++;
				break;
			}
		}

		avg_count++;

		if (skip)
			continue;

		// fft
		for(int i=0; i<n; i++)
		{
			fft_in[i][0] = data[i][0] / 32767.0 * fft_window_real[i];
			fft_in[i][1] = data[i][1] / 32767.0 * fft_window_real[i];

// 			in[i][0] = sin((i%4)*PI) * fft_window[i];
// 			in[i][1] = cos((i%4)*PI) * fft_window[i];
		}

		fftwf_execute(plan_complex);

		for(int i=0; i<n; i++)
		{
			float v = fft_out[i][0]*fft_out[i][0]+fft_out[i][1]*fft_out[i][1];
			damp[i] += v;
		}

	}

 	avg_count-=skip_count;

	fclose(f);

	double peak_v = 0;
	int peak_p;
	for(int i=0; i<n; i++)
	{
		if (damp[i] > peak_v)
		{
			peak_v = damp[i];
			peak_p = i;
		}
	}

	float peak_db = sqrt(peak_v / avg_count)/n;
	peak_db = log10(peak_db) * 20;

	peak_v = 0;
	for(int i=-window_spreading; i<= window_spreading; i++)
		peak_v += damp[(peak_p+i)%n];

	float peak_dbs = sqrt(peak_v / avg_count)/n;
	peak_dbs = log10(peak_dbs) * 20;

	f = fopen(file_out, "wb");
	fprintf(f, "offset,amp(dbc)\n");
	int peak_index = peak_p >= n/2 ? peak_p-n : peak_p;
	float peak_freq = sample_rate * (float)peak_index / n;

	int index_bw = n*(float)bw/sample_rate;


	for(int i=1; i<index_bw; i+=max(1.0,i*0.02))
	{
		int index = peak_p +i;
		if (index > N)
			index -= N;

		float freq_offset = (float)i*sample_rate/n;
		float dbc = log10(damp[index] / peak_v)*10 - freq_res_dbhz;
		fprintf(f, "%f,%f\n", freq_offset, dbc);

	}
	fclose(f);

	double signal_amp = 0;
	double noise_amp = 0;
	int peak_signed = peak_p > n/2 ? peak_p-n : peak_p;
	for(int i=n-1; i>=0; i--)
	{
		int i_signed = i > n/2 ? i-n : i;
		float freq_offset = (float)abs(i_signed-peak_signed)*sample_rate/n;

		if (freq_offset > signal_bw/2)
			noise_amp += damp[i];
		else
		{
			signal_amp += damp[i];
		}
	}

	float signal_amp_db = log10(sqrt(signal_amp/avg_count)/n)*20;

	float SNR = log10(signal_amp/noise_amp)*10;

	set_fft_size(2048);

	return 0;
}

void noise_rms()
{
	FILE * f = fopen("E:\\pcm42222.pcm", "rb");

	if(!f)
		return;
	
	double avg[2] = {0};
	int count = 0;

	while (!feof(f))
	{
		float d[2];
		fread(d, 1, sizeof(d), f);
		avg[0] += d[0];
		avg[1] += d[1];
		count ++;
	}

	avg[0] /= count;
	avg[1] /= count;

	double sq[2] = {0};

	fseek(f, 0, SEEK_SET);
	while (!feof(f))
	{
		float d[2];
		fread(d, 1, sizeof(d), f);
		sq[0] += (d[0]-avg[0])*(d[0]-avg[0]);
		sq[1] += (d[1]-avg[1])*(d[1]-avg[1]);
	}

	sq[0] /= count;
	sq[1] /= count;

	float dbrms0 = log10(sq[0]) * 10;
	float dbrms1 = log10(sq[1]) * 10;

	printf("%.2f/%.2f db RMS, %f/%f avg", dbrms0, dbrms1, avg[0], avg[1]);


}

int noise_rmsi()
{
	FILE * f = fopen("D:\\temp\\log.wav", "rb");

	if(!f)
		return 0;

	fseek(f, 100, SEEK_END);
	int count = ftell(f)/2;
	int16_t *p = new int16_t[count-50];
	fseek(f, 100, SEEK_SET);
	fread(p, 2, count, f);
	fclose(f);

	int64_t avg = 0;
	for(int i=0; i<count; i+=2)	// skip zeros
		avg += p[i];

	float avgf = (double)avg/(count/2);

	double rms = 0;
	for(int i=0; i<count; i+=2)
	{
		float v = p[i] - avgf;
		rms += v*v;
	}

	rms /= count/2;
	rms = sqrt(rms);

	printf("avg:%.3f, rms:%.3f(%.1fdbFS)\n", avgf, rms, log10(rms/32768)*20);

	return 0;
}

int16_t tmp[102400];
int _10k()
{

	FILE * f = fopen("Z:\\10k.pcm", "wb");
	for(int i=0; i<102400; i++)
		tmp[i] = sin(i*2*PI/4.8f)*32767;
	fwrite(tmp, 2, 102400, f);
	fclose(f);

	return 0;
}

int DLIA_test()
{
	FILE * f = fopen("Z:\\100nf.wav", "rb");
	if (!f)
		return 0;

	fseek(f, 0, SEEK_END);
	int count = ftell(f)/8;
	fseek(f, 0, SEEK_SET);
	
	float *p = new float[count*2];
	fread(p, 8, count, f);

	
	float phase = 0;
	float freq = 1000;
	float sample_rate = 48000;
	float delta_phase = freq * 2*PI / sample_rate;
	int cycle = sample_rate / freq;

	float VI = 0;
	float VQ = 0;
	float II = 0;
	float IQ = 0;

	count = count / cycle * cycle;
	count = cycle*4;

	for(int i=0; i<count; i++)
	{
		phase += delta_phase; 
		if (phase > 2*PI)
			phase -= 2*PI;

		float s = sin(phase);
		float c = cos(phase);

		VI += c*p[i*2+0];
		VQ += s*p[i*2+0];

		II += c*p[i*2+1];
		IQ += s*p[i*2+1];
	}

	float phaseV = atan2(VI, VQ);
	float phaseI = atan2(-II, -IQ);

	float phaseV180 = phaseV * 180 / PI;
	float phaseI180 = phaseI * 180 / PI;

	float dp = radian_sub(phaseV, phaseI);
	float dp90 = radian_sub(-PI/2, dp);
	float D = tan(dp90);
	float Q =1/D;
	float dp_deg = dp * 180 / PI;
	float dtime_ns = 1/freq * dp;

	delete p;
	return 0;
}

int getTimePeriod()
{

	int l = timeGetTime();
	while(1)
	{
		//Sleep(1);
		
		int l2 = timeGetTime();
		if (l2 != l)
		{
			printf("\r%dms", l2 - l);
			l = l2;
		}
	}

	return 0;
}

int clock_vs_ldo()
{
	FILE * noise = fopen("C:\\Users\\my12doom\\Desktop\\ldo_noise.pcm", "rb");
	FILE * out = fopen("C:\\Users\\my12doom\\Desktop\\clock.pcm", "wb");

	fseek(noise, 0, SEEK_END);
	int count = ftell(noise)/4;
	fseek(noise, 0, SEEK_SET);

	for(int i=0; i<count; i++)
	{
		float n;
		fread(&n, 1, 4, noise);
		
		float v = sin(i*1000*2*PI / 48000) * (n+1);

		fwrite(&v, 1, 4, out);
		fwrite(&n, 1, 4, out);
	}

	fclose(noise);
	fclose(out);
	
	return 0;
}


static void
_hsv2rgb(float *rgb, float *hsv)
{
	int i;
	float h, s, v;
	float r, g, b;
	float f, p, q, t;

	/* Unpack input */
	h = hsv[0];
	s = hsv[1];
	v = hsv[2];

	if( s <= 0.0f ) {
		/* achromatic (grey) */
		r = g = b = v;
		goto done;
	}

	h *= 5.0f;			/* sector 0 to 5 */
	i = floor(h);
	f = h - i;			/* fractional part of h */
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );

	switch (i % 6) {
		case 0:
			r = v;
			g = t;
			b = p;
			break;
		case 1:
			r = q;
			g = v;
			b = p;
			break;
		case 2:
			r = p;
			g = v;
			b = t;
			break;
		case 3:
			r = p;
			g = q;
			b = v;
			break;
		case 4:
			r = t;
			g = p;
			b = v;
			break;
		case 5:
		default:
			r = v;
			g = p;
			b = q;
			break;
	}

done:
	/* Pack results */
	rgb[0] = r;
	rgb[1] = g;
	rgb[2] = b;
}

void set_hsv(RGBQUAD *p, float h, float s, float v)
{
	float hsv[3] = {h, s, v};
	float rgb[4];

	_hsv2rgb(rgb, hsv);
	p->rgbRed = rgb[0]*255;
	p->rgbGreen = rgb[1]*255;
	p->rgbBlue = rgb[2]*255;
	p->rgbReserved = 255;
}

int load_color_table()
{
	int N = 256;
	int i;
	int m = N >> 4;

	for (i=0; i<m; i++)
	{
		float p = (1.0f * i) / (N - 1);

		set_hsv(&color_tbl[i],
			0.90f,				/* H */
			0.50f,				/* S */
			0.15f + 4.0f * p		/* V */
			);
	}

	for (i=m; i<N; i++)
	{
		float p = (1.0f * i) / (N - 1);

		set_hsv(&color_tbl[i],
			0.80f -   p * 0.80f,
			1.00f - ((p < 0.85f) ? 0.0f : ((p - 0.85f) * 3.0f)),
			0.60f + ((p < 0.40f) ? p : 0.40f)
			);
	}

	return 0;
}

/*
// https://en.wikipedia.org/wiki/Polynomial_regression
int poly_fit()
{
	const int maxN = 4000;
	real_t xx[maxN];
	real_t yy[maxN];
	int N = 0;

	FILE * f = fopen("C:\\Users\\mo\\Desktop\\data.txt", "rb");
	while (!feof(f))
	{
		char tmp[1024] = {0};
		fgets(tmp, 1024, f);
		float freq, vco, cap, amp;
		if (sscanf(tmp, "%f\t%f\t%f\t%f", &freq, &vco, &cap, &amp) == 4)
		{
			xx[N] = freq;
			yy[N] = cap;
			N++;
		}
	}
	fclose(f);

	// 	float p[order] = {0.5, 1, 3};
	// 	for(int i=0; i<N; i++)
	// 	{
	// 		real_t x = i+1;
	// 		xx[i] = x;
	// 		yy[i] = p[0] + p[1]*x + p[2] * x * x;
	// 	}
	// 
	const int order = 3;
	real_t v[order*order] = {0};

	matrix XTX(order, order, v);
	matrix XTy(order, 1, v);

	for(int i=0; i<N; i++)
	{
		real_t xn = 1;
		XTy[0] += yy[i];
		for(int j=1; j<order; j++)
		{
			xn *= xx[i];
			v[j] += xn;
			XTy[j] += xn * yy[i];
		}
		for(int j=order; j<order*2-1; j++)
		{
			xn *= xx[i];
			v[j] += xn;
		}
	}

	XTX(0,0) = N;
	XTX(order-1,order-1) = v[order*2-2];
	for(int j=1; j<order; j++)
	{
		for(int k=0; k<j+1; k++)
		{
			XTX(k, j-k) = v[j];
			XTX(order-1-k, order-1-(j-k)) = v[order*2-2-j];
		}
	}

	matrix est = XTX.inversef() * XTy;
	float c0 = est[0];
	float c1 = est[1];
	float c2 = est[2];

	float R2 = 0;
	for(int i=0; i<N; i++)
	{
		float x = xx[i];
		float y = (c0 + x*c1 + x*x*c2 );
		float res2 = (y-yy[i])*(y-yy[i]);
		assert(res2<=1.1f);
		R2 += res2;
	}
	R2 /= N;
	float R = sqrt(R2);

	return 0;
}
*/

int main(int argc, char* argv[])
{
	timeBeginPeriod(1);
// 	test_sinc();
//    	AMgen();
// 	for(int i=0; i<50; i++)
	//draw_wave();
	
// 	return 0;
// 	noise_rmsi();


//   	noise_rms();
//  	phase_noise2();

	load_color_table();
	set_fft_size(N);
	open_device(device_list[4]);
	DialogBoxW(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, main_window_proc);
	clearup();


	return 0;
}


