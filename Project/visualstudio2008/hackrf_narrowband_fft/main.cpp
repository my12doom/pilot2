#include <Windows.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "fftw/fftw3.h"
#include "sse_mathfun.h"
#include "autolock.h"
#include "resource.h"
#include <float.h>
#include <CommCtrl.h>

#include "device_hackrf.h"
#include "device_bulk.h"
#include "device_dummy.h"

#include "filter1d.h"

#include <intrin.h>
#include <tmmintrin.h>
#include "FFTProcessor.h"


#pragma comment(lib, "pthreadVSE2.lib")
#pragma comment(lib, "fftw/libfftw3f-3.lib")
#pragma comment(lib, "fftw/libfftw3-3.lib")
using namespace NBFFT;

// configuration
char *rbw_list[] = {"1Khz", "10Khz", "100Khz", "1Mhz", "custom", "FFT bins"};
char *device_list[] = {"hackrf", "dummy", "bulk"};
const int minN = 1024;
const int maxN = 1024*1024;

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
int int8tofloat(float *dst, const int8_t*src, float scale, int count);
int int16tofloat(float *dst, const int16_t*src, float scale, int count);
float total(const float *data, int count);
void total_complex(const float *data, float *out, int count);
int set_fft_size(int N);
int open_device(const char *name, void *extra = NULL);

// variables
static const double PI = acos(-1.0);
float range_low = -130;
float range_high = 0;
filter1d rbw_filter;
rbw_type g_rbw_type = rbw_gaussion;

device *d = NULL;

float sample_rate;
float bw_per_bin;
float rbw = 10000;

int N = minN*2;
fftwf_complex * in = NULL;//
fftwf_complex * fft_out = NULL;
fftwf_plan p = NULL;
float ampsq[maxN];
float peaksq_amp = 0;
float fft_window[maxN];
float fft_window_SIMD[maxN*2];
float DC[2] = {0};
_critical_section cs_amp;
_critical_section cs_rbw;
_critical_section cs_fft;
_critical_section cs_device;
_critical_section cs_outfile;
HWND slider;
HWND slider2;
float phase_imba = 0.0f;
float gain_imba = 1.0f;

FILE * f = NULL;
int canvas_width = N;
int canvas_height;

int process_fft_block(float *ampsq_copy, void *buf, sample_quant type)
{
	float dt = N/sample_rate;
	float alpha_attack = dt / (dt + 1.0f/(150*PI * 1));
	float alpha_release = dt / (dt + 1.0f/(5*PI * 1));

	__m128 mattack_a = _mm_set1_ps(alpha_attack);
	__m128 mrelease_a = _mm_set1_ps(alpha_release);
	__m128 m1attack_a = _mm_sub_ps(_mm_set1_ps(1), mattack_a);
	__m128 m1release_a = _mm_sub_ps(_mm_set1_ps(1), mrelease_a);
	__m128 mN = _mm_set1_ps(1.0/N/N);
	__m128 mff;
	_mm_storeu_si128((__m128i*)&mff, _mm_set1_epi8(0xff));

	// convert/copy data
	if (0 == type)
		int8tofloat((float*)in, (int8_t*)buf, 1/128.0f, N*2);
	else if (1 == type)
		int16tofloat((float*)in, (int16_t*)buf, 1/32768.0f, N*2);
	else if (2 == type)
		memcpy(in, buf, N*sizeof(float)*2);

	// find DC offset
	float DC_N[2] = {0};
	total_complex((float*)in, DC_N, N);

	float alpha_DC = dt / (dt + 1.0f/(0.5f*PI * 1));
	DC[0] = DC[0] * (1-alpha_DC) + DC_N[0] * alpha_DC / N;
	DC[1] = DC[1] * (1-alpha_DC) + DC_N[1] * alpha_DC / N;

	// find peak
	__m128 mpeak = _mm_set1_ps(0);		
	for(int i=0; i<N*2; i+=8)
	{
		float *p = (float *)in;
		__m128 m1 = _mm_loadu_ps(p+i);
		m1 = _mm_mul_ps(m1, m1);
		__m128 m2 = _mm_loadu_ps(p+i+4);
		m2 = _mm_mul_ps(m2, m2);
		m1 = _mm_hadd_ps(m1, m2);
		mpeak = _mm_max_ps(mpeak, m1);
	}
	float *pk = (float*)&mpeak;
	peaksq_amp *= pow(1-alpha_release, N/1000);
	peaksq_amp = max(max(pk[0], pk[1]), max(pk[2], pk[3]));

	/*
	// IQ phase miss-match correction, see https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms1-ebz/iq_correction
	float tanm = tan(phase_imba);
	float secm = 1.0f/cos(phase_imba);
	for(int i=0; i<N; i++)
	{
		in[i][0] *= gain_imba;

		in[i][1] = tanm * in[i][0] + secm * in[i][1];
	}
	*/

	// apply DC offset correction and fft window
	__m128 mdc = _mm_set_ps(DC[1], DC[0], DC[1], DC[0]);

	for(int i=0; i<N*2; i+=4)
	{
		__m128 ma = _mm_loadu_ps(((float *)in)+i);
		__m128 mb = _mm_loadu_ps(fft_window_SIMD+i);
		ma = _mm_sub_ps(ma, mdc);
		ma = _mm_mul_ps(ma, mb);
		_mm_storeu_ps(((float *)in)+i, ma);
	}


	fftwf_execute(p);

	for(int j=0; j<N; j+=4)
	{
		__m128 m1 = _mm_loadu_ps((float*)&fft_out[j]);
		__m128 m2 = _mm_loadu_ps((float*)&fft_out[j+2]);

		m1 = _mm_mul_ps(m1, m1);
		m2 = _mm_mul_ps(m2, m2);
		m1 = _mm_hadd_ps(m1, m2);
		m1 = _mm_mul_ps(m1, mN);

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

	return 0;
}

uint8_t *gbuf = new uint8_t[4*maxN*2];
int gbuf_count = 0;
// len:length in elements
// type: 0: int8_t, 1:int16_t, 2:float
int rx(void *buf, int len, sample_quant type)
{
	float dt = N/sample_rate;
	float alpha_release = dt / (dt + 1.0f/(5*PI * 1));

	float *ampsq_copy = NULL;
	
	int count = len/2;
	int element_size = (0 == type) ? 1 : ((1==type) ? 2 : 4);
	cs_outfile.enter();
	if (f)
		fwrite(buf, 1, len*element_size, f);
	cs_outfile.leave();


	int8_t *buf8 = (int8_t*)buf;

	cs_fft.enter();
	while (gbuf_count == 0 && count >= N)
	{
		if (!ampsq_copy)
		{
			ampsq_copy = new float[N];
			cs_amp.enter();
			memcpy(ampsq_copy, ampsq, N*sizeof(float));
			cs_amp.leave();
		}

		process_fft_block(ampsq_copy, buf8, type);
		count -= N;
		buf8 += element_size*2*N;
	}

	while(count)
	{
		int blk_count = min(N - gbuf_count, count);
		memcpy(gbuf+element_size*gbuf_count*2, buf8, blk_count*element_size*2);

		gbuf_count += blk_count;
		count -= blk_count;
		buf8 += blk_count*element_size*2;

		if (N == gbuf_count)
		{
			if (!ampsq_copy)
			{
				ampsq_copy = new float[N];
				cs_amp.enter();
				memcpy(ampsq_copy, ampsq, N*sizeof(float));
				cs_amp.leave();
			}
			process_fft_block(ampsq_copy, gbuf, type);
			gbuf_count = 0;
		}
	}

	/*
	for(int i=0; i<len/2 && (len/2-i) >= N; i+= N)
	{
		process_fft_block(ampsq_copy, buf8+i*2*element_size, type);
	}
	*/

	if (ampsq_copy)
	{
		cs_amp.enter();
		memcpy(ampsq, ampsq_copy, N*sizeof(float));
		cs_amp.leave();

		delete [] ampsq_copy;
	}

	cs_fft.leave();

	return 0;
}


RGBQUAD canvas[1024*1024];

int draw_line(int x1, int y1, int x2, int y2, RGBQUAD color)	// draw line in pixel space
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

int draw(HWND drawing_hwnd)
{
	RECT rect;
	GetClientRect(drawing_hwnd, &rect);
	HDC hdc = GetDC(drawing_hwnd);
	HDC memDC = CreateCompatibleDC(hdc);
	HBITMAP bitmap = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
	HGDIOBJ obj = SelectObject(memDC, bitmap);

	// drawing region
	int width = rect.right - rect.left;
	int height = rect.bottom - rect.top;
	canvas_width = width;
	canvas_height = height;
	memset(canvas, 0xff, sizeof(canvas));
	RGBQUAD red = {0,0,255,255};
	RGBQUAD blue = {255,0,0,255};
	RGBQUAD light_red = {180,180,255,255};

	// axies
	for(int i=range_low; i<=range_high; i+=10)
	{
		int y = height * (range_high - i) / (range_high-range_low);
		for(int j=0; j<width; j++)
			canvas[y*width+j] = light_red;
	}


	// data
	cs_device.enter();
	if (!d)
	{
		cs_device.leave();
		return 0;
	}
	sample_type type = d->get_sample_type();
	cs_device.leave();

	float amp_pixel[2048];
	cs_amp.enter();
	memcpy(amp, ampsq+N/2, N/2*sizeof(float));
	memcpy(amp+N/2, ampsq, N/2*sizeof(float));		// reordered for applying RBW filter
	cs_amp.leave();

	if (type == real_sample)
		memset(amp, 0, sizeof(float)*N/2);
	
	if (g_rbw_type != rbw_none)
		rbw_filter.apply(amp, N);

	for(int i=0; i<width; i++)
	{
		int n;
		if (type == complex_sample)
		{
			if (i<=width/2)
				n = i*N/2/(width/2);							// negative frequency
			else
				n = (i-width/2-1)*(N/2-1)/(width/2-1) + N/2;	// positive part
		}
		else
		{
			n = i*(N/2-1)/(width-1) + N/2;	// positive part
		}
		
		float db = (amp[n]>0) ? log10(amp[n]) * 10 : range_low;

		if (db >= range_low && db <= range_high)
		{
			amp_pixel[i] = height * (range_high - db) / (range_high-range_low);
		}
		else if (db > range_high)
			amp_pixel[i] = 0;
		else if (db < range_low)
			amp_pixel[i] = height-1;
		else
			amp_pixel[i] = -1;
	}

	for(int i=1; i<width; i++)
	{
		draw_line(i-1, amp_pixel[i-1], i, amp_pixel[i], blue);
	}

	// overall amplitude
	float peakamp_db = min(max(range_low,log10(peaksq_amp)*10), range_high);
 	draw_line(0, height, 0, height * (range_high - peakamp_db) / (range_high-range_low), red);

	// transfer
	SetBitmapBits(bitmap, rect.right * rect.bottom * 4, canvas);
	BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);

	DeleteObject(obj);
	DeleteObject(bitmap);
	DeleteDC(memDC);
	ReleaseDC(drawing_hwnd, hdc);

	return 0;
}

int init_dialog(HWND hDlg)
{
//	window_start = GetTickCount();
	SetTimer(hDlg, 0, 16, NULL);
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
	for(int n = minN; n<=maxN; n*=2)
	{
		char tmp[20];
		sprintf(tmp, "%d", n);
		SendMessageA(fft, CB_ADDSTRING, 0, (LPARAM)tmp);
	}
	SendMessageA(fft, CB_SETCURSEL, 1, 0);

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
	
	return set_rbw_filter_hz(rbw_gaussion, n, -range_low);
}

int update_fft_size(HWND cb)
{
	char tmp[100] = {0};
	GetWindowTextA(cb, tmp, sizeof(tmp)-1);

	return set_fft_size(atoi(tmp));
}

INT_PTR CALLBACK main_window_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	switch( msg ) 
	{
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);

			if (id == IDC_RECORD)
			{
				cs_outfile.enter();
				if (f)
				{
					fclose(f);
					f = NULL;
					SetDlgItemTextA(hDlg, IDC_RECORD, "Record");
				}
				else
				{
					f = fopen("E:\\log2.pcm", "wb");
					SetDlgItemTextA(hDlg, IDC_RECORD, "Stop Recording");
				}
				cs_outfile.leave();
			}

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
			}

			if (id == IDC_RBW && HIWORD(wParam) == EN_CHANGE)
			{
				char tmp[100] = {0};
				GetDlgItemTextA(hDlg, IDC_RBW, tmp, sizeof(tmp)-1);
				float hz = atof(tmp);
				if (hz>0)
					set_rbw_filter_hz(rbw_gaussion, atoi(tmp), -range_low);
			}
		}
		break;


	case WM_INITDIALOG:
		init_dialog(hDlg);
		break;
	case WM_HSCROLL:
		phase_imba = (SendMessage(slider, TBM_GETPOS, 0, 0)-500)*2*PI/180/500;
		gain_imba = (SendMessage(slider2, TBM_GETPOS, 0, 0)-500)*0.05f/500 + 1.0f;
		break;

	case WM_CLOSE:		
		EndDialog(hDlg, 0);
		break;
	case WM_TIMER:
		draw(GetDlgItem(hDlg, IDC_GRAPH));
		{
			//set_rbw_filter(rbw_gaussion, SendMessage(slider, TBM_GETPOS, 0, 0)+1, 100);
			float peakamp_db = log10(peaksq_amp)*10 -3;	// -3: 2channel
			char tmp[30];
			sprintf(tmp, "%.1fdbFS, 10db/div", peakamp_db);
			SetWindowTextA(hDlg, tmp);

		}
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
		for(int i=0; i<=taps_half; i++)
		{
			rbw_taps[i+taps_half] = exp(-a*i*i);
			rbw_taps[taps_half-i] = rbw_taps[i+taps_half];
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
	rbw = bw3db_hz;
	bw_per_bin = sample_rate / N;

	if (bw3db_hz < 1)
	{
		g_rbw_type = rbw_none;
		printf("RBW filter disabled\n");

		return 0;
	}

	if (bw_per_bin > bw3db_hz)
		printf("too narrow rbw requested: %.1f, but minimum is %.1f hz\n", bw3db_hz, bw_per_bin);	

	return set_rbw_filter_bins(type, bw3db_hz/bw_per_bin, 120);
}

#define _mm_cmpge_epu8(a, b) _mm_cmpeq_epi8(_mm_max_epu8(a, b), a)
#define _mm_cmpge_epu16(a, b) _mm_cmpeq_epi16(_mm_max_epu16(a, b), a)

int int8tofloat(float *dst, const int8_t*src, float scale, int count)
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

int int16tofloat(float *dst, const int16_t*src, float scale, int count)
{
	__m128 mscale = _mm_set1_ps(scale);
	__m128i m8000 = _mm_set1_epi16(0x8000);
	for(int i=0; i<count; i+=8)
	{
		__m128i m1 = _mm_loadu_si128((__m128i*)(src+i));

		__m128i mh = _mm_cmpge_epu16(m1, m8000);
		__m128i m2 = _mm_unpackhi_epi16(m1, mh);
		m1 = _mm_unpacklo_epi16(m1, mh);

		__m128 m1f = _mm_mul_ps(_mm_cvtepi32_ps(m1), mscale);
		__m128 m2f = _mm_mul_ps(_mm_cvtepi32_ps(m2), mscale);

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
	N = _N;
	if (p)
		fftwf_destroy_plan(p);
	if (in)
		fftwf_free(in);
	if (fft_out)
		fftwf_free(fft_out);

	in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
	fft_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
	p = fftwf_plan_dft_1d(N, in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);

	// blackman-harris window function
	float a0 = 0.35875;
	float a1 = 0.48829;
	float a2 = 0.14128;
	float a3 = 0.01168;
	for(int i=0; i<N; i++)
	{
		fft_window[i] = (a0 - a1*cos(2*PI*i/N) + a2*cos(4*PI*i/N) - a3*cos(6*PI*i/N))/a0;	// /a0: normalized
		fft_window_SIMD[i*2] = fft_window_SIMD[i*2+1] = fft_window[i];
	}

	// small initial value
	for(int i=0; i<N; i++)
	{
		ampsq[i] = 1e-15;
	}
	cs_fft.leave();

	set_rbw_filter_hz(g_rbw_type, rbw, -range_low);


	return 0;
}

int open_device(const char *name, void *extra /*= NULL*/)
{
	cs_device.enter();
	if (d)
	{
		d->destroy();
		delete d;
	}

	if (strcmp(name, "hackrf") == 0)
	{
		d = new NBFFT::hackrf_device;
		d->init(rx);		
	}

	if (strcmp(name, "bulk") == 0)
	{
		d = new NBFFT::bulk_device;
		d->init(rx);		
	}

	if (strcmp(name, "dummy") == 0)
	{
		d = new NBFFT::dummy_device;
		d->init(rx);
	}

	sample_rate = d->get_sample_rate();
	range_high = 0;
	range_low = -d->dynamic_range_db();
	cs_device.leave();
	set_fft_size(N);


	return 0;
}

int main(int argc, char* argv[])
{
	set_fft_size(N);
	open_device(device_list[0]);
	DialogBoxW(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, main_window_proc);

	if (d)
	{
		d->destroy();
		delete d;
	}

	fftwf_destroy_plan(p);
	fftwf_free(in);
	fftwf_free(fft_out);


	return 0;
}


