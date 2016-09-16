#include <HAL/Resources.h>
#include <HAL/SIL_WIN32/Win32Timer.h>
#include <stdio.h>
#include <utils/param.h>
#include <Windows.h>

using namespace SIL_WIN32;

void reset_system()
{
	printf("system reset requested!!!\n");
	exit(1);
}

const char bsp_name[] = " SIL_WIN32";

int bsp_init_all()
{
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);

	static Win32Timer timer[3];
	manager.register_Timer("mainloop", &timer[0]);
	manager.register_Timer("log", &timer[1]);
	manager.register_Timer("imu", &timer[2]);

	timer[0].set_period(THREAD_PRIORITY_NORMAL);
	timer[1].set_period(THREAD_PRIORITY_IDLE);
	timer[2].set_period(THREAD_PRIORITY_HIGHEST);

	param("time", 3000)=1000;
	param("ekf", 0)=2;

	return 0;
}