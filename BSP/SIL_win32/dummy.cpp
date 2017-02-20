#include <HAL/Resources.h>
#include <HAL/SIL_WIN32/Win32Timer.h>
#include <HAL/SIL_WIN32/Win32UART.h>
#include <HAL/sensors/EBusIn.h>
#include <stdio.h>
#include <utils/param.h>
#include <Windows.h>

using namespace SIL_WIN32;
using namespace sensors;

void reset_system()
{
	printf("system reset requested!!!\n");
	exit(1);
}

const char bsp_name[] = " SIL_WIN32";

Win32UART uart_rc;
EBusIN ebus;

int init_rc()
{
	if (uart_rc.init(39) < 0)
	{
		printf("failed opening uart 39\n");
		return -1;
	}

	ebus.init(&uart_rc);
	manager.register_RCIN(&ebus);

	return 0;
}

int bsp_init_all()
{
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);
	init_rc();

	static Win32Timer timer[3];
	manager.register_Timer("mainloop", &timer[0]);
	manager.register_Timer("log", &timer[1]);
	manager.register_Timer("imu", &timer[2]);

	timer[0].set_period(THREAD_PRIORITY_NORMAL);
	timer[1].set_period(THREAD_PRIORITY_IDLE);
	timer[2].set_period(THREAD_PRIORITY_HIGHEST);

	param("time", 3000)=3000;
	param("ekf", 0)=2;

	return 0;
}