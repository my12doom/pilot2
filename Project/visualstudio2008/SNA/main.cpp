#include <stdio.h>
#include <windows.h>
#include "resource.h"
#include "enumser.h"
#include <HAL/SIL_WIN32/Win32UART.h>

using namespace SIL_WIN32;

int draw(HWND drawing_hwnd);
int update_cursor_marker();

HWND hwnd;
Win32UART uart;
bool opened = false;

const int N_PTS = 2000;

float zeros[N_PTS];
float freqs[N_PTS];
float values[N_PTS];
int point_count = 0;

float m1 = -1;
float value_m1 = -9999;

int get_sel_com()
{
	HWND combo = GetDlgItem(hwnd, IDC_COM);
	char tmp[200] = {0};
	GetWindowTextA(combo, tmp, sizeof(tmp)-1);
	return atoi(tmp+3);
}

char line[99999] = {0};
DWORD WINAPI rx_thread(LPVOID p)
{
	int counter = 0;
	int update_counter = 0;
	while (1)
	{
		if (opened)
		{
			memset(line, 0, sizeof(line));
			if (uart.readline(line, sizeof(line)-1) > 0)
			{
				printf(line);

				if (counter < N_PTS && strstr(line, "\t"))
				{
					float freq = atof(line);
					float v = atof(strstr(line, "\t")+1);

					freqs[counter] = freq;
					values[counter] = v;

					counter ++;
				}

				if (strstr(line, "0\n") == line)
				{
					printf("UP");
					update_counter ++;
					point_count = counter;
					counter = 0;

					SetDlgItemTextA(hwnd, IDC_WORKING, update_counter & 1 ? "*" : "**");
				}
			}
			else
			{
				Sleep(1);
			}
		}
		else
		{
			Sleep(1);
		}
	}
}

INT_PTR CALLBACK main_window_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	static int window_start = GetTickCount();
	switch( msg ) 
	{
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			int not = HIWORD(wParam);

			if (id == IDC_OPENCLOSE)
			{
				if (!opened)
				{
					int comid = get_sel_com();
					printf("COM %d\n", comid);

					opened = uart.init(comid) == 0;
					uart.set_baudrate(57600);
				}
				else
				{
					uart.close();
					opened = false;
				}

				SetDlgItemTextA(hwnd, IDC_OPENCLOSE, opened ? "Close" : "Open");
			}

			if (id == IDC_UP)
			{
				if (opened)
				{
					char cmd[200] = {0};
					GetDlgItemTextA(hwnd, IDC_FREQ, cmd, sizeof(cmd)-1);					
					uart.write(cmd, strlen(cmd));
				}
			}

			if (id == IDC_ZERO)
			{
				memcpy(zeros, values, sizeof(zeros));
			}

			if (id == IDC_COM && not == CBN_DROPDOWN)
			{
				CPortAndNamesArray ports;

				QueryUsingSetupAPI(GUID_DEVINTERFACE_COMPORT, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE, ports);

				HWND combo = GetDlgItem(hDlg, IDC_COM);
				int comid = get_sel_com();
				SendMessageA(combo, CB_RESETCONTENT , 0, 0);
				int sel = -1;
				int n = 0;
				for(std::vector<std::pair<UINT, std::string> >::iterator i = ports.begin(); i!= ports.end(); ++i)
				{
					char tmp[100];
					sprintf(tmp, "COM%d(%s)", (*i).first, (*i).second.c_str());
					SendMessageA(combo, CB_ADDSTRING, 0, (LPARAM)tmp);

					if ((*i).first == comid)
						sel = n;
					n++;
				}

				if (sel>=0)
					SendMessageA(combo, CB_SETCURSEL, sel, 0);

			}
			//QEC_enabled = SendMessage(GetDlgItem(hDlg, IDC_QEC), BM_GETCHECK, 0, 0);
		}
		break;

	case WM_MOUSEMOVE:
	case WM_LBUTTONDOWN:
		update_cursor_marker();
		break;


	case WM_INITDIALOG:
		{
			hwnd = hDlg;
			SetTimer(hDlg, 0, 100, NULL);
			CreateThread(0, 0, rx_thread, 0, 0, 0);
			SetDlgItemTextA(hwnd, IDC_FREQ, "freq,60,6000,100\n");
		}

		break;

	case WM_TIMER:
		draw(GetDlgItem(hDlg, IDC_GRAPH));
		break;

	case WM_CLOSE:
		EndDialog(hDlg, 0);
		break;

	default:
		return FALSE;
	}

	return TRUE; // Handled message
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine,int nShowCmd)
{
	DialogBoxW(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, main_window_proc);



	return 0;
}


int main()
{
	return WinMain(0, 0, 0, 0);
}


RGBQUAD canvas[1024*1024];
int canvas_width;
int canvas_height;

int draw_point(int x0, int y0, int size, RGBQUAD color)
{
	for(int y=y0-size/2; y<y0+size; y++)
		for(int x=x0-size/2; x<x0+size; x++)
			if (y>=0 && y<canvas_height && x>=0 && x<canvas_width)
				canvas[y*canvas_width+x] = color;

	return 0;
}

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
	update_cursor_marker();

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


	// pull data snapshot
	float _freq[N_PTS];
	float _values[N_PTS];
	float _zeros[N_PTS];
	int count = point_count;
	//cs.enter();
	memcpy(_freq, freqs, sizeof(freqs));
	memcpy(_values, values, sizeof(freqs));
	memcpy(_zeros, zeros, sizeof(freqs));
	//cs.leave();

	for(int i=0; i<N_PTS; i++)
		_values[i] = _zeros[i] - _values[i];
	float span = _freq[count-1] - _freq[0];

	RGBQUAD red = {0,0,255,255};
	RGBQUAD blue = {255,0,0,255};
	RGBQUAD black = {0,0,0,255};
	RGBQUAD light_red = {180,180,255,255};
	RGBQUAD yellow = {0,255,255,255};


	float range_low = -800;
	float range_high = 600;
	float range_step = 100;

	// axies
	for(int i=range_low; i<=range_high; i+= range_step)
	{
		int y = height * (range_high - i) / (range_high-range_low);
		for(int j=0; j<width; j++)
			canvas[y*width+j] = light_red;
	}
	// zero axie
	int y = height * range_high / (range_high-range_low);
	for(int j=0; j<width; j++)
		canvas[y*width+j] = black;

	// data
	int x1 = 0;
	int y1 = height/2;
	for(int i=1; i<count; i++)
	{
		int x = (_freq[i] - _freq[0]) * width / span;

		// "warning zone"
		int yh = height * (range_high - (_zeros[i] - 500)) / (range_high-range_low);
		int yl = height * (range_high - (_zeros[i] - 1200)) / (range_high-range_low);
		draw_line(x, yh, x, 0, yellow);
		draw_line(x, yl, x, height, yellow);

		// data
		int y =  height * (range_high - _values[i]) / (range_high-range_low);
		draw_line(x1, y1, x, y, blue);
		draw_point(x, y, 3, blue);
		x1 = x;
		y1 = y;

	}


	SetBitmapBits(bitmap, rect.right * rect.bottom * 4, canvas);

	BitBlt(hdc, 0, 0, rect.right, rect.bottom, memDC, 0, 0, SRCCOPY);

	DeleteObject(obj);
	DeleteObject(bitmap);
	DeleteDC(memDC);
	ReleaseDC(drawing_hwnd, hdc);

	return 0;
}


int update_cursor_marker()
{
	POINT p;
	GetCursorPos(&p);
	HWND graph = GetDlgItem(hwnd, IDC_GRAPH);
	ScreenToClient(graph, &p);

	// pull data snapshot
	int count = point_count;
	if (point_count <= 0)
		return -1;

	float _freq[N_PTS];
	float _values[N_PTS];
	float _zeros[N_PTS];
	//cs.enter();
	memcpy(_freq, freqs, sizeof(freqs));
	memcpy(_values, values, sizeof(freqs));
	memcpy(_zeros, zeros, sizeof(freqs));
	//cs.leave();

	float span = _freq[count-1] - _freq[0];
	for(int i=0; i<N_PTS; i++)
		_values[i] = _zeros[i] - _values[i];

	// interpolate mouse & marker
	static float freq_mouse = 0;
	static float value_mouse = -9999;
	if (p.x >= 0 && p.x < canvas_width && p.y >= 0 && p.y < canvas_height)
	{
		freq_mouse = p.x * span / canvas_width + _freq[0];
		// update_marker if mouse down
		if (GetKeyState(VK_LBUTTON)<0)
			m1 = freq_mouse;
		for(int i=1; i<count; i++)
		{
			if (_freq[i-1] <= freq_mouse && freq_mouse <= _freq[i])
			{
				float scale1 = freq_mouse - _freq[i-1];
				float scale2 = _freq[i] - freq_mouse;


				value_mouse = (_values[i-1]*scale2 + _values[i]*scale1) / (scale1+scale2);
				break;
			}
		}
	}

	// interpolate marker value
	if (m1 > 0)
	for(int i=1; i<count; i++)
	{
		if (_freq[i-1] <= m1 && m1 <= _freq[i])
		{
			float scale1 = m1 - _freq[i-1];
			float scale2 = _freq[i] - m1;


			value_m1 = (_values[i-1]*scale2 + _values[i]*scale1) / (scale1+scale2);
			break;
		}
	}

	// update display
	char tmp[100];
	float slope = 22;
	sprintf(tmp, "mouse, %.1fMhz, %.2fdb\n M1:%.1fMhz, %.2fdb", freq_mouse, value_mouse/slope, m1, value_m1/slope);
	SetDlgItemTextA(hwnd, IDC_MARKER, tmp);

	return 0;
}
