#include "DSOwindow.h"
#include "resource.h"
#include <stdint.h>
#include <assert.h>
#include <intrin.h>
#include <tmmintrin.h>
#include "wfm_draw.h"
#include <math.h>
#include <CommCtrl.h>
#include <windowsx.h>
#include "memory_pool.h"
#include "streaming-load-memcpy.h"

HWND dso = NULL;
HWND hdso_graph;
HWND slider_intensity;
HANDLE hDSO_thread;
HANDLE hextract_thread;
MagicRingBuffer DSO_ringbuf(1024*8192*4);
_critical_section cs_DSO_ringbuf;
bool dso_exiting = false;
bool drawer_exiting = false;
extern int N;
const int maxN = 4096*1024;
threaded_drawer *drawer = NULL;
float wfms = 0;
float trigs = 0;
int intense = 10;
int16_t threshold = 0;
static int frm_width = 1024;
static int frm_height = 720;
static int sample_per_pixel = 32;
static int zoom = 1;
bool run = true;
bool g_single = false;
bool auto_trigger = true;

bool last_is_single = false;
DWORD last_drop_time = timeGetTime();
bool graph_update = false;
extern float sample_rate;

void extract(int trig_count);
void init_dialog();

static int limit(int v, int low, int high)
{
	if (v < low)
		return low;
	if (v > high)
		return high;
	return v;
}


INT_PTR CALLBACK dso_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	switch( msg ) 
	{
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			if (id == IDC_ZOOM_IN)
			{
				if (sample_per_pixel == 1)
					zoom *= 2;
				else
					sample_per_pixel /= 2;
			}
			if (id == IDC_ZOOM_OUT)
			{
				if (zoom > 1)
					zoom /= 2;
				else
					sample_per_pixel *= 2;
			}

			sample_per_pixel = limit(sample_per_pixel, 1, 1024);
			zoom = limit(zoom, 1, 128);

			if (id == IDC_RUN)
			{
				run = !run;
				g_single = false;
			}

			if (id == IDC_Single)
			{
				g_single = true;
				run = true;
			}

			auto_trigger = BST_CHECKED == SendMessage(GetDlgItem(hDlg, IDC_AUTO), BM_GETCHECK, 0, 0);
		}
		break;
	case WM_INITDIALOG:
		{
			dso = hDlg;
			init_dialog();
			SetTimer(hDlg, 0, 100, NULL);
		}
		break;

	case WM_LBUTTONDOWN:
		{
			POINT mouse;
			GetCursorPos(&mouse);
			ScreenToClient(hdso_graph, &mouse);
			if (mouse.y >= 0 && mouse.y < frm_height)
				threshold = mouse.y * -65535 / frm_height  + 32768;

			graph_update = true;
		}
		break;

	case WM_MOUSEHWHEEL:
		{
			int xPos = GET_X_LPARAM(lParam);
			int yPos = GET_Y_LPARAM(lParam);
			int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
			int v = zDelta / WHEEL_DELTA;
			
			sample_per_pixel *= pow(2.0f, v);

			if (sample_per_pixel > 128)
				sample_per_pixel = 128;
			if (sample_per_pixel < 1)
				sample_per_pixel = 1;
		}
		break;

	case WM_CLOSE:
		exit_dso();
		KillTimer(hDlg, 0);
		EndDialog(hDlg, -1);
		break;

	case WM_TIMER:
		char tmp[100];
		sprintf(tmp, "wfm/s:%d/%d, %d pt (%.3fus), %.2f/%.2f Mpt/s", int(wfms), int(trigs), sample_per_pixel * frm_width, sample_per_pixel * frm_width * 1e6 / sample_rate,
			(float)sample_per_pixel * frm_width * wfms * 1e-6, (float)sample_per_pixel * frm_width * trigs * 1e-6);
		if (zoom > 1)
			sprintf(tmp+strlen(tmp), ", %dX", zoom);
		if (last_drop_time > timeGetTime() - 500)
			strcat (tmp, " ***SLOW CPU***");
		SetWindowTextA(dso, tmp);
		SetDlgItemTextA(dso, IDC_RUN, run ? "Stop" : "Run");
		break;

	case WM_HSCROLL:
		intense = pow(1.03f, (WORD)SendMessage(slider_intensity, TBM_GETPOS, 0, 0));
		break;

	default:
		return FALSE;
	}

	return TRUE; // Handled message
}

void init_dialog()
{
	hdso_graph = GetDlgItem(dso, IDC_DSO);
	slider_intensity = GetDlgItem(dso, IDC_INTENSE);
	SendMessage(slider_intensity, TBM_SETRANGEMIN, TRUE, 1);
	SendMessage(slider_intensity, TBM_SETRANGEMAX, TRUE, 255);
	SendMessage(slider_intensity, TBM_SETPOS, TRUE, 125);
	intense = pow(1.03f, (WORD)SendMessage(slider_intensity, TBM_GETPOS, 0, 0));
	SendMessage(GetDlgItem(dso, IDC_AUTO), BM_SETCHECK, auto_trigger?BST_CHECKED:BST_UNCHECKED, 0);

	ShowWindow(GetDlgItem(dso, IDC_EXTRACT), SW_HIDE);
	SetDlgItemTextA(dso, IDC_RUN, run ? "Stop" : "Run");
	RECT rect;
	GetClientRect(hdso_graph, &rect);

	frm_width = rect.right - rect.left;
	frm_height = rect.bottom - rect.top;
}

DWORD WINAPI DSO_draw_thread(LPVOID p)
{
	HANDLE thread = GetCurrentThread();
	int last_frm_count = 0;
	
	float *frm = new float[frm_width*frm_height+16];
	RGBQUAD *graph = new RGBQUAD[frm_width*frm_height+16];

	HDC hdc = GetDC(hdso_graph);
	HDC memDC = CreateCompatibleDC(hdc);
	HBITMAP bitmap = CreateCompatibleBitmap(hdc, frm_width, frm_height);
	HGDIOBJ obj = SelectObject(memDC, bitmap);

	float max_raw = 0;
	while (!drawer_exiting)
	{
		int l = timeGetTime();
		if (!drawer)
		{
			Sleep(1);
			continue;
		}

		int count = drawer->extract(frm);
		bool sleep = false;
		if (count == 0)
		{
			sleep = true;
			count = last_frm_count;
		}
		else
		{
			graph_update = true;
			last_frm_count = count;
		}

		if (count < 500 && !last_is_single)
			count = 500;

		float intense_f = intense == 255 ? 6500 : intense;


		if (graph_update)
		{
			max_raw = 0;
			for(int y=0; y<frm_height; y++)
			{
				uint16_t tmp[5120] = {0};
				for(int x=0; x<frm_width; x++)
				{
					float raw = frm[y+frm_height*x];
// 					max_raw = max(raw, max_raw);
					float v = raw * 250 * intense_f / count / sqrt((float)sample_per_pixel/zoom);
					tmp[x] = v < 65535 ? v : 65535;

					RGBQUAD &p = graph[y*frm_width+x];
					p.rgbReserved = 0xff;
					p.rgbBlue = 0;
					p.rgbRed = sqrt((float)tmp[x]);
					p.rgbGreen = sqrt((float)tmp[x]);

				}
			}

// 			printf("max_raw = %.1f\n", max_raw);

			// trigger line
			for(int x=0; x<frm_width; x++)
			{
				int y = frm_height-1 - ((threshold+32768) * frm_height / 65536);
				RGBQUAD &p = graph[y*frm_width+x];
				p.rgbReserved = 0xff;
				p.rgbBlue = 0;
				p.rgbRed = 120;
				p.rgbGreen = 120;
			}

			for(int y=0; y<frm_height; y++)
			{
				RGBQUAD &p = graph[y*frm_width+frm_width/2];
				p.rgbReserved = 0xff;
				p.rgbBlue = 0;
				p.rgbRed = 120;
				p.rgbGreen = 120;
			}
			SetBitmapBits(bitmap, frm_width * frm_height * 4, graph);
			BitBlt(hdc, 0, 0, frm_width, frm_height, memDC, 0, 0, SRCCOPY);

			graph_update = false;
		}





		int draw_time = timeGetTime()-l;

//   		printf("draw time:%d\n", draw_time);
		if (sleep)
			Sleep(draw_time);
	}

	DeleteObject(obj);
	DeleteObject(bitmap);
	DeleteDC(memDC);
	ReleaseDC(hdso_graph, hdc);

	delete frm;
	delete graph;
	//delete last_frm;

	return 0;
}

int16_t wave[1024*1024];
DWORD WINAPI DSO_trigger_thread(LPVOID p)
{
	uint8_t *buf = new uint8_t[4*maxN*2];


	int fps = 30;
	int hysteric = 256;
	int samples_used = 0;
	int auto_trigger_timeout = sample_rate * 0.5;

	threaded_drawer drawer2(frm_width, frm_height, 3);
	drawer = &drawer2;

	enum trigger_type
	{
		rising,
		falling,
		rising_falling,
	} trigger = rising;

	enum trigger_state
	{
		low,
		high,
		initial
	} state = initial;

	int trig_count = 0;
	int draw_count = 0;
	int scan_count = 0;
	int l = GetTickCount();

	int16_t *data;
	int data_count = 0;
	bool have_trig = false;

	int auto_trigger_counter = auto_trigger_timeout;

	while (!dso_exiting)
	{
		int threshold_high = threshold+hysteric;
		int threshold_low = threshold-hysteric;
		int samples_per_wave = frm_width * sample_per_pixel;
		int max_data_per_cycle = max(samples_per_wave*3/2, sample_rate / fps / 3);

		// get data
		{
			_autolock lck(&cs_DSO_ringbuf);
			data = (int16_t*)DSO_ringbuf.get_read_ptr();
			data_count = min(DSO_ringbuf.used_space() / 2, max_data_per_cycle);


			if (!run && data_count > 0)
			{
				DSO_ringbuf.pop_front(NULL, data_count * 2);
				continue;
			}
		}

		if (data_count < samples_per_wave)
		{
			Sleep(1);
			continue;
		}



		int triggered_count = 0;
		have_trig = false;
		state = initial;
	
		bool _single = g_single;

		for(int i=samples_per_wave/2; i<data_count - samples_per_wave/2; i++)
		{
			trigger_state state1 = state;

			// generate trig
			scan_count ++;
			bool trig = false;
			int16_t d = data[i];
			if (d < threshold_low)
			{
				if (state == high && (trigger == falling || trigger == rising_falling))
					trig = true;
				state = low;
			}

			if (d > threshold_high)
			{
				if (state == low && (trigger == rising || trigger == rising_falling))
					trig = true;
				state = high;
			}

			if (!auto_trigger || _single)
				auto_trigger_counter = samples_per_wave;

			if (trig && _single)
			{
				drawer->commit();
				drawer->wait_for_idle();
				drawer->snapshot();
				ResumeThread(hextract_thread);
			}

			// check trig && draw
			if (trig || (auto_trigger_counter == 0 && !_single))
			{
				int16_t d1 = data[i-1];

				if (drawer2.draw16((int16_t*)(data + i - samples_per_wave/2), samples_per_wave, NULL, zoom, threshold) == 0)
					draw_count++;
				else
					last_drop_time = timeGetTime();

				triggered_count = i + samples_per_wave/2;

				i += samples_per_wave;

				state = initial;
				trig_count ++;
				have_trig = true;
				last_is_single = _single;

				if (trig)
					auto_trigger_counter = auto_trigger_timeout;
				else
					auto_trigger_counter = samples_per_wave;

				if (_single && trig)
				{
					run = false;
					g_single = false;
					drawer->snapshot();
					break;
				}

			}
			else
			{
				auto_trigger_counter --;
			}
		}

		// consumed data
		cs_DSO_ringbuf.enter();
		int to_remove = max(data_count - samples_per_wave, triggered_count);
		if (to_remove > 0)
		{
			int o = DSO_ringbuf.pop_front(NULL, to_remove * 2);
			assert(o > 0);
		}
		cs_DSO_ringbuf.leave();

		// snapshoting
		drawer2.commit();
		samples_used += to_remove;
		if (samples_used >= sample_rate / fps)
		{
			samples_used -= sample_rate / fps;


			if (trig_count > 0)
			{
				drawer2.snapshot();
				ResumeThread(hextract_thread);
			}

			int dt = timeGetTime() - l;
			dt = max(1, dt);
//  			printf("\n%d/%d trig/s\n", draw_count*1000/dt, trig_count*1000/dt);

			float alpha = 0.95f;
			wfms = wfms * alpha + (1-alpha) *(draw_count*1000/dt);
			trigs = trigs * alpha + (1-alpha) *(trig_count*1000/dt);
			trig_count = 0;
			draw_count = 0;
			l = timeGetTime();
		}

	}

	delete buf;
	drawer = NULL;

	return 0;
}

int show_dso_window(HINSTANCE instance, HWND parent)
{
	if (dso == NULL)
	{
		dso_exiting = false;
		drawer_exiting = false;
		dso = CreateDialog(instance, MAKEINTRESOURCE(IDD_DSO), parent, (DLGPROC)dso_proc); 
		hDSO_thread = CreateThread(NULL, NULL, DSO_trigger_thread, NULL, 0, 0);
		hextract_thread = CreateThread(NULL, NULL, DSO_draw_thread, NULL, 0, 0);
	}

	ShowWindow(dso, SW_SHOW);

	return 0;
}

int exit_dso()
{
	if (dso)
	{
		drawer_exiting = true;
		ResumeThread(hextract_thread);
		WaitForSingleObject(hextract_thread, INFINITE);
		dso_exiting = true;
		WaitForSingleObject(hDSO_thread, INFINITE);
		dso = NULL;
	}

	return 0;
}
