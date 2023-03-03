#include "sweepSA.h"
#include "resource.h"
#include <Windows.h>
#include <windowsx.h>
#include "device.h"
#include "autolock.h"
#include "fftw/fftw3.h"
#include "interpolate_1d.h"
#include <Protocol/common.h>
#include <intrin.h>
#include <tmmintrin.h>

using namespace NBFFT;
int FFT_SIZE = 8192;

HWND sweepSA = NULL;
HANDLE hdrawerThread = NULL;
extern _critical_section cs_device;
extern device *d;
sweep_config config = {5725e6, 5e6, 20};
int subband_count;
static int byte_per_sample;
static sample_quant quant = sample_8bit;

static fftwf_complex * fft_in = NULL;
static fftwf_complex * fft_out = NULL;
static fftwf_plan plan_complex = NULL;


int sweep_handler(void *buf, int len, sweep_info info);
int int8tofloat(float *dst, const int8_t*src, int count, float scale=1/128.0f);
int int16tofloat(float *dst, const int16_t*src, int count, float scale = 1/32768.0f);
int data_handler(void *buf, int len);

static float amp[2048*16384];
static float ampsq[2048*16384];
static float amp_max[2048*16384];
static float amp_min[2048*16384];
static float amp_avg[2048*16384];

#define countof(s) (sizeof(s)/sizeof(s[0]))

FILE * ff = fopen("D:\\temp\\1.pcm", "wb");
FILE * xls = fopen("D:\\temp\\1.csv", "wb");
static float fft_window_complex[65536];
static float fft_window_real[65536];

int sweep_handler(void *buf, int len, sweep_info info)
{
	int freq_index = (info.center_freq - config.freq_start) / config.freq_step;
// 	printf("f:%.1f, idx=%d\n", info.center_freq/1e6, freq_index);
	if (freq_index == 0)
	{
		static int l = GetTickCount();
		printf("cycle time:%dms\n", GetTickCount()-l);
		l = GetTickCount();
	}


// 	if (ff && freq_index == 1)
// 	{
// 		static uint8_t *buf2 = new uint8_t[FFT_SIZE*4];
// 		__m128i m128 = _mm_set1_epi8(128);
// 
// 		for(int i=0; i<len; i+=16)
// 		{
// 			__m128i m = _mm_loadu_si128((__m128i*)(((int8_t*)buf)+i));
// 			m = _mm_add_epi8(m, m128);
// 			_mm_storeu_si128((__m128i*)(((int8_t*)buf2)+i), m);
// 		}
// 		fwrite(buf2, 1, len, ff);
// 
// 	}

	// signed 8bit complex block
	memset(fft_in, 0, sizeof(fftwf_complex) * FFT_SIZE);

	if (quant == sample_8bit)
		int8tofloat((float*)fft_in, (int8_t*)buf, len);
	else
		int16tofloat((float*)fft_in, (int16_t*)buf, FFT_SIZE*2);

// 	if (ff)
// 	{
// 		fwrite(fft_in, FFT_SIZE*8, 1, ff);
// 	}

	// apply fft window
	for(int i=0; i<FFT_SIZE*2; i+=4)
	{
		__m128 ma = _mm_loadu_ps(((float *)fft_in)+i);
		__m128 mb = _mm_loadu_ps(fft_window_complex+i);
		//ma = _mm_sub_ps(ma, mdc);
		ma = _mm_mul_ps(ma, mb);
		_mm_storeu_ps(((float *)fft_in)+i, ma);
	}

	fftwf_execute(plan_complex);
	float n2 = 1.0f / FFT_SIZE / FFT_SIZE;



// printf("%d\n", freq_index);

	for(int i=1; i<3; i++)
	{
		for(int j=0; j<FFT_SIZE/subband_count; j++)
		{
			int fft_idx = i*FFT_SIZE/subband_count + j;
			int tbl_index = freq_index * FFT_SIZE/2 - FFT_SIZE / 4 + fft_idx;

			fft_idx -= FFT_SIZE/2;

			bool dc_reject = false;//abs(fft_idx) < 3;

			if (fft_idx > FFT_SIZE-1)
				fft_idx -= FFT_SIZE;
			if (fft_idx < 0)
				fft_idx += FFT_SIZE;

			if (tbl_index >0 && tbl_index < countof(ampsq))
			{
				float r = fft_out[fft_idx][0];
				float im = fft_out[fft_idx][1];

				if (dc_reject)
					r = im = 1e-9;

// 				if (fft_idx % 32 == 0)
// 					printf("%d\n", fft_idx);
				float sq = r*r+im*im;
				ampsq[tbl_index] = sq * n2;
				amp[tbl_index] = sqrt(ampsq[tbl_index]);
			}
		}
	}

// 	if (freq_index == 1)
// 	{
// 		for (int i = -20; i<=20; i++)
// 		{
// 			int idx = i> 0 ? i : i+FFT_SIZE/2;
// 			fprintf(xls, "%.1f,", log10(ampsq[idx])*10);
// 		}
// 		fprintf(xls, "\n");
// 		fflush(xls);
// 	}

// 	printf("%lld\n", info.center_freq);
	return 0;
}

static void init_dialog()
{
	_autolock lck(&cs_device);
	config.freq_step = d->get_sample_rate()/2;
	d->init(sweep_handler, config);
	subband_count = 4;//d->get_sample_rate() / config.freq_step;
	byte_per_sample = d->get_sample_quant() == sample_16bit ? 2 : 1;
	quant = d->get_sample_quant();

	fft_in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*FFT_SIZE);
	fft_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*FFT_SIZE);
	plan_complex = fftwf_plan_dft_1d(FFT_SIZE, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);

// 	// blackman-harris window function
// 	float a0 = 0.35875;
// 	float a1 = 0.48829;
// 	float a2 = 0.14128;
// 	float a3 = 0.01168;
// 	float a4 = 0;

	//flat-top window function
	float a0 = 0.21557895;
	float a1 = 0.41663158;
	float a2 = 0.277263158;
	float a3 = 0.083578947;
	float a4 = 0.006947368;

	for(int i=0; i<FFT_SIZE; i++)
	{
		fft_window_real[i] = (a0 - a1*cos(2*PI*i/FFT_SIZE) + a2*cos(4*PI*i/FFT_SIZE) - a3*cos(6*PI*i/FFT_SIZE) + a4*cos(8*PI*i/FFT_SIZE))/a0;	// /a0: normalized
		fft_window_complex[i*2] = fft_window_complex[i*2+1] = fft_window_real[i];
	}

}



INT_PTR CALLBACK sweepSAproc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	switch( msg ) 
	{
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
		}
		break;
	case WM_INITDIALOG:
		{
			sweepSA = hDlg;
			init_dialog();
			SetTimer(hDlg, 0, 100, NULL);
		}
		break;

	case WM_LBUTTONDOWN:
		{
		}
		break;

	case WM_MOUSEHWHEEL:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
			int v = zDelta / WHEEL_DELTA;

		}
		break;

	case WM_CLOSE:
		exit_sweepSA();
		KillTimer(hDlg, 0);
		break;

	case WM_TIMER:
		break;

	case WM_HSCROLL:
		break;

	default:
		return FALSE;
	}

	return TRUE; // Handled message
}

static int draw_line(RGBQUAD*canvas, int x1, int y1, int x2, int y2, RGBQUAD color, int canvas_width = 1024, int canvas_height = 1024)	// draw line in pixel space
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


static RGBQUAD graph[2048*2048];
DWORD WINAPI sweepSA_drawer(LPVOID p)
{
	HWND hSAgraph = GetDlgItem(sweepSA, IDC_GRAPH);
	RECT rect;
	GetClientRect(hSAgraph, &rect);
	int frm_width = rect.right - rect.left;
	int frm_height = rect.bottom - rect.top;
	memset(graph, 0x20, sizeof(RGBQUAD) * frm_width * frm_height);
	float amp_pixel[3][2048];

	float range_low = -110;
	float range_high = -10;
	RGBQUAD red = {0,0,255,255};
	RGBQUAD blue = {255,0,0,255};
	RGBQUAD light_red = {180,180,255,255};
	RGBQUAD yellow = {0,255,255,255};
	int width = frm_width;
	int height = frm_height;

	int n = 0;
	while (sweepSA)
	{
		Sleep(10);
		memset(graph, n, sizeof(RGBQUAD) * frm_width * frm_height);

		HDC hdc = GetDC(hSAgraph);
		HDC memDC = CreateCompatibleDC(hdc);
		HBITMAP bitmap = CreateCompatibleBitmap(hdc, frm_width, frm_height);
		HGDIOBJ obj = SelectObject(memDC, bitmap);


		// Y axies
		for(int i=range_low; i<=range_high; i+=10)
		{
			int y = height * (range_high - i) / (range_high-range_low);
			for(int j=0; j<width; j++)
				graph[y*width+j] = light_red;
		}

		// X axies
		for(int x=0; x<width; x+=width/10)
		{
			for(int y=0; y<height; y++)
				graph[y*width+x] = light_red;
		}

		int bin_count = FFT_SIZE/2 * config.points;

		create_mipmap1d_minmax(ampsq, amp_max, amp_min, bin_count);
		create_mipmap1d(ampsq, amp_avg, bin_count);
		interpolate1d_mipmap(ampsq, amp_pixel[0], bin_count, frm_width, amp_min);
		interpolate1d_mipmapmax(ampsq, amp_pixel[1], bin_count, frm_width, amp_max);
		interpolate1d_mipmap(ampsq, amp_pixel[2], bin_count, frm_width, amp_avg);

		// draw lines
		float last_min = -100;
		float last_max = -100;
		RGBQUAD yellow = {0,255,255,255};

		float last_avg = height-1;
		for(int i=0; i<frm_width; i++)
		{
			float dbavg = (amp_pixel[2][i]>0) ? log10(amp_pixel[2][i]) * 10 : range_low;
			float dbmin = (amp_pixel[0][i]>0) ? log10(amp_pixel[0][i]) * 10 : range_low;
			float dbmax = (amp_pixel[1][i]>0) ? log10(amp_pixel[1][i]) * 10 : range_low;

			dbavg = limit(dbavg, range_low, range_high);
			dbmin = limit(dbmin, range_low, range_high);
			dbmax = limit(dbmax, range_low, range_high);

			float vavg = (height-1) * (range_high - dbavg) / (range_high-range_low);
			float vmin = (height-1) * (range_high - dbmin) / (range_high-range_low);
			float vmax = (height-1) * (range_high - dbmax) / (range_high-range_low);


			draw_line(graph, i, vmin, i, vmax, yellow, width, height);
			if (i>0)
			{
				draw_line(graph, i, vavg, i-1, last_avg, blue, width, height);
			}
			last_avg = vavg;
		}


		SetBitmapBits(bitmap, frm_width * frm_height * 4, graph);
		BitBlt(hdc, 0, 0, frm_width, frm_height, memDC, 0, 0, SRCCOPY);
		DeleteObject(obj);
		DeleteObject(bitmap);
		DeleteDC(memDC);
		ReleaseDC(hSAgraph, hdc);
	}

	return 0;
}

int show_sweepSA_window(HINSTANCE instance, HWND parent)
{
	if (sweepSA == NULL)
	{
		sweepSA = CreateDialog(instance, MAKEINTRESOURCE(IDD_SWEEPSA), parent, (DLGPROC)sweepSAproc);
		hdrawerThread = CreateThread(NULL, NULL, sweepSA_drawer, sweepSA, NULL, NULL);
	}

	ShowWindow(sweepSA, SW_SHOW);

	return 0;
}

int exit_sweepSA()
{
	if (sweepSA)
	{
		{
		_autolock lck(&cs_device);
		if (d)
			d->init(data_handler);
		}

		EndDialog(sweepSA, 0);
		sweepSA = NULL;
		WaitForSingleObject(hdrawerThread, INFINITE);

	}

	return 0;
}
