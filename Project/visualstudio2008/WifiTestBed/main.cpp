#include <Windows.h>
#include <Commctrl.h>
#include "resource.h"
#include "http_cmd.h"
#include <stdio.h>
#include <string.h>
#include "socket3333.h"
#include "http_cmd.h"
#include <conio.h>

#pragma comment(lib, "Comctl32.lib")

HWND wnd;
bool ok3333 = false;
bool okwifi = false;

DWORD WINAPI thread3333(LPVOID p)
{
	while(true)
	{
		Sleep(10);

		ok3333 = last_packet_time() < 3000;

		SetDlgItemTextA(wnd, IDC_YAP,  ok3333 ? "飞控已连接": "飞控未连接");
		UpdateWindow(GetDlgItem(wnd, IDC_YAP));

		if (last_packet_time() > 3000)
			connect();
	}

	return 0;
}

DWORD WINAPI thread_wifi(LPVOID p)
{
	while(true)
	{
		Sleep(10);

		okwifi = test80("192.168.1.254") == 0;

		SetDlgItemTextA(wnd, IDC_WIFI,  okwifi ? "WIFI已连接": "WIFI已跪");
	}

	return 0;
}

bool all_running = false;

DWORD WINAPI all_thread(LPVOID p)
{
	while(true)
	{
		if (!all_running)
		{
			Sleep(10);
			continue;
		}

		DWORD id[] = {IDC_FLASHLIGHT_ON, IDC_FLASHLIGHT_OFF, IDC_TAKEOFF, IDC_DISARM, IDC_RED, IDC_GREEN, IDC_BLUE, IDC_000, IDC_VIDEO};
		for(int i=0; i<sizeof(id)/4; i++)
		{
			SendMessageA(wnd, WM_COMMAND, id[i], 0);
			Sleep(1000);
		}

		all_running = false;
	}
}


INT_PTR CALLBACK WndProcTestbed(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		wnd = hWnd;
		CreateThread(NULL, NULL, thread3333, NULL, NULL, NULL);
		CreateThread(NULL, NULL, thread_wifi, NULL, NULL, NULL);
		CreateThread(NULL, NULL, all_thread, NULL, NULL, NULL);
		break;
	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);
			if (id == IDC_VIDEO)
			{
				char path[MAX_PATH] = {0};
				GetModuleFileNameA(NULL, path, MAX_PATH-1);
				strcpy((char*)strrchr(path, '\\'), "\\ffplay.exe");
				ShellExecuteA(NULL, "open", path, "-rtsp_transport tcp rtsp://192.168.1.254:554/xxx.mov", NULL, SW_SHOWNORMAL);
			}

			else if (id == IDC_RED)
			{
				http_cmd("custom=1&cmd=8003&str=RffG00B00D31");
			}
			else if (id == IDC_GREEN)
			{
				http_cmd("custom=1&cmd=8003&str=R00GFFB00D31");
			}
			else if (id == IDC_BLUE)
			{
				http_cmd("custom=1&cmd=8003&str=R00G00BFFD31");
			}
			else if (id == IDC_000)
			{
				http_cmd("custom=1&cmd=8003&str=R00G00B00D30");
			}

			else if (id == IDC_FLASHLIGHT_ON)
			{
				cmd("flashlight,1\n");
			}
			else if (id == IDC_FLASHLIGHT_OFF)
			{
				cmd("flashlight,0\n");
			}
			else if (id == IDC_TAKEOFF)
			{
				cmd("takeoff\n");
			}
			else if (id == IDC_DISARM)
			{
				cmd("disarm\n");
			}

			else if (id == IDC_ALL_RUN)
			{
				all_running = true;
			}
		}
		break;

	case WM_CTLCOLORSTATIC:
		{
			static HBRUSH hBrush = CreateSolidBrush(RGB(230,230,230));

			DWORD CtrlID = GetDlgCtrlID((HWND)lParam); //Window Control ID
			
			if (CtrlID == IDC_YAP) //If desired control
			{
				HDC hdcStatic = (HDC) wParam;
				SetTextColor(hdcStatic, RGB(0,0,0));
				SetBkColor(hdcStatic, RGB(ok3333 ? 0 : 255, ok3333 ? 255 : 0, 0));
				return (INT_PTR)hBrush;
			}
			if (CtrlID == IDC_WIFI) //If desired control
			{
				HDC hdcStatic = (HDC) wParam;
				SetTextColor(hdcStatic, RGB(0,0,0));
				SetBkColor(hdcStatic, RGB(okwifi ? 0 : 255, okwifi ? 255 : 0, 0));
				return (INT_PTR)hBrush;
			}
		}

	default:
		return FALSE;
	}
	return TRUE;
}

int main()
{

	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(1, 1), &wsaData) != 0) 
	{
		fprintf(stderr, "WSAStartup failed.\n");
		getch();
		return -1;
	}

	InitCommonControls();
	DialogBox(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, WndProcTestbed);
	return 0;
}
