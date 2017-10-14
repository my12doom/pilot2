#include <Windows.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "fftw/fftw3.h"
#include "Fourier.h"
#include "sse_mathfun.h"
#include "autolock.h"
#include "resource.h"
#include <float.h>

#ifdef WIN32
#include <Windows.h>
#include <intrin.h>
#include <tmmintrin.h>
#else
#include <x86intrin.h>
#include <tmmintrin.h>
#endif

#pragma comment(lib, "pthreadVSE2.lib")
#pragma comment(lib, "fftw/libfftw3f-3.lib")

// #define  hackrf
#ifdef hackrf
float sample_rate = 20E6;
#include "device_hackrf.h"
#define device_init device_hackrf_init
#define device_exit device_hackrf_exit
#else
float sample_rate = 2E6;
#include "device_bulk.h"
#define device_init device_bulk_init
#define device_exit device_bulk_exit
#endif

#define N 1024
fftwf_complex * in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
fftwf_complex * fft_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*N);
fftwf_plan p;
float log_amp[N];
_critical_section cs;

FILE * f = fopen("Z:\\log2.pcm", "wb");
int canvas_width = N;
int canvas_height;

// len:length in elements
// type: 0: int8_t, 1:int16_t
int rx(void *buf, int len, int type)
{
	if (f)
		fwrite(buf, 1, len*(1+type), f);

	float dt = N/sample_rate;
	float alpha_attack = dt / (dt + 1.0f/(1500*PI * 1));
	float alpha_release = dt / (dt + 1.0f/(0.2*PI * 1));

	__m128 mattack_a = _mm_set1_ps(alpha_attack);
	__m128 mrelease_a = _mm_set1_ps(alpha_release);
	__m128 m1attack_a = _mm_sub_ps(_mm_set1_ps(1), mattack_a);
	__m128 m1release_a = _mm_sub_ps(_mm_set1_ps(1), mrelease_a);
	__m128 mN = _mm_set1_ps(1.0/N/N);
	__m128 m20 = _mm_set1_ps(1.0/log(10.0)*10);
	__m128 mff;
	_mm_storeu_si128((__m128i*)&mff, _mm_set1_epi8(0xff));

	for(int i=0; i<len/2; i+= N)
	{
		if (0 == type)
		{
			int8_t *p = (int8_t*)buf;
			for(int j=0; j<N; j++)
			{
				in[j][0] = p[(i+j)*2+0]/128.0f;
				in[j][1] = p[(i+j)*2+1]/128.0f;
			}
		}
		else if (1 == type)
		{
			int16_t *p = (int16_t*)buf;
			for(int j=0; j<N; j++)
			{
				in[j][0] = p[(i+j)*2+0]/32767.0f;
				in[j][1] = p[(i+j)*2+1]/32767.0f;
			}
		}
		fftwf_execute(p);

		float log_amp_copy[N];
		cs.enter();
		memcpy(log_amp_copy, log_amp, sizeof(log_amp_copy));
		cs.leave();
		for(int j=0; j<N; j+=4)
		{
			__m128 m1 = _mm_loadu_ps((float*)&fft_out[j]);
			__m128 m2 = _mm_loadu_ps((float*)&fft_out[j+2]);

			m1 = _mm_mul_ps(m1, m1);
			m2 = _mm_mul_ps(m2, m2);
			m1 = _mm_hadd_ps(m1, m2);
			m1 = _mm_mul_ps(m1, mN);

			m1 = log_ps(m1);
			m1 = _mm_mul_ps(m1, m20);

			__m128 pre = _mm_loadu_ps(log_amp_copy+j);
			__m128 mattack = _mm_mul_ps(m1, mattack_a);
			mattack = _mm_add_ps(mattack, _mm_mul_ps(pre, m1attack_a));
			__m128 mrelease = _mm_mul_ps(m1, mrelease_a);
			mrelease = _mm_add_ps(mrelease, _mm_mul_ps(pre, m1release_a));

			m2 = _mm_cmpge_ps(m1, pre);
			__m128 m2n = _mm_xor_ps(m2, mff);
			mattack = _mm_and_ps(mattack, m2);
			mrelease = _mm_and_ps(mrelease, m2n);

			m1 = _mm_or_ps(mattack, mrelease);

			_mm_storeu_ps(log_amp_copy+j, m1);
		}

		cs.enter();
		memcpy(log_amp, log_amp_copy, sizeof(log_amp_copy));
		for(int i=0; i<N; i++)
			if (_isnan(log_amp[i]) || i == 0)		// remove DC offset and NANs from log_ps error.
				log_amp[i] = _isnan(log_amp[(i+1)%N]) ? -999 : log_amp[(i+1)%N];
		cs.leave();
	}

	return 0;
}


RGBQUAD canvas[1024*1024];

int draw_line(int x1, int y1, int x2, int y2, RGBQUAD color)	// draw line in pixel space
{
	int dx = x2 - x1;
	int dy = y2 - y1;
	int ux = ((dx > 0) << 1) - 1;//x的增量方向，取或-1
	int uy = ((dy > 0) << 1) - 1;//y的增量方向，取或-1
	int x = x1, y = y1, eps;//eps为累加误差

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

int draw(HWND drawing_hwnd)
{
	RECT rect;
	GetClientRect(drawing_hwnd, &rect);
	HDC hdc = GetDC(drawing_hwnd);
	HDC memDC = CreateCompatibleDC(hdc);
	HBITMAP bitmap = CreateCompatibleBitmap(hdc, rect.right, rect.bottom);
	HGDIOBJ obj = SelectObject(memDC, bitmap);

	// draw
	int width = rect.right - rect.left;
	int height = rect.bottom - rect.top;
	canvas_width = width;
	canvas_height = height;
	memset(canvas, 0xff, sizeof(canvas));
	float amp[N];
	float amp_pixel[2048];
	cs.enter();
	memcpy(amp, log_amp, sizeof(log_amp));
	cs.leave();

	RGBQUAD blue = {255,0,0,255};
	RGBQUAD light_red = {180,180,255,255};


	float range_low = -110;
	float range_high = 0;
	for(int i=range_low; i<=range_high; i+=10)
	{
		int y = height * (range_high - i) / (range_high-range_low);
		for(int j=0; j<width; j++)
			canvas[y*width+j] = light_red;
	}
	for(int i=0; i<width; i++)
	{
		int n;
		if (i<=width/2)
			n = i*N/2/(width/2) + N/2-1;			// negative frequency
		else
			n = (i-width/2-1)*(N/2-1)/(width/2-1);	// positive part

		if (amp[n] >= range_low && amp[n] <= range_high)
		{
			amp_pixel[i] = height * (range_high - amp[n]) / (range_high-range_low);
		}
		else if (amp[n] > range_high)
			amp_pixel[i] = 0;
		else if (amp[n] < range_low)
			amp_pixel[i] = height-1;
	}

	for(int i=1; i<width; i++)
	{
		draw_line(i-1, amp_pixel[i-1], i, amp_pixel[i], blue);
	}

	SetBitmapBits(bitmap, rect.right * rect.bottom * 4, canvas);

	BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);

	DeleteObject(obj);
	DeleteObject(bitmap);
	DeleteDC(memDC);
	ReleaseDC(drawing_hwnd, hdc);

	return 0;
}

INT_PTR CALLBACK main_window_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	static int window_start = GetTickCount();
	switch( msg ) 
	{
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
		}
		break;


	case WM_INITDIALOG:
		window_start = GetTickCount();
		SetTimer(hDlg, 0, 16, NULL);
		break;

	case WM_CLOSE:		
		EndDialog(hDlg, 0);
		break;
	case WM_TIMER:
		draw(GetDlgItem(hDlg, IDC_GRAPH));
		break;

	default:
		return FALSE;
	}

	return TRUE; // Handled message
}


int main(int argc, char* argv[])
{
	for(int i=0; i<N; i++)
		log_amp[i] = -999;

 	p = fftwf_plan_dft_1d(N, in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
	int res = device_init(rx);
	if (res < 0)
	{
		return -1;
	}

	DialogBoxW(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, main_window_proc);

	device_exit();

	fftwf_destroy_plan(p);
	fftwf_free(in);
	fftwf_free(fft_out);


	return 0;
}


