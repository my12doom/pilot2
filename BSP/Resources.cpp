#include <BSP/Resources.h>
#include <stdlib.h>
#include <string.h>
using namespace HAL;
using namespace STM32F4;
//Manager::LED part
int  Manager::Register_LED(const char *name,LED *pLED)
{
	strcpy(led_table[led_num].name,name);
	led_table[led_num].num=led_num;
	led_table[led_num].pLED=pLED;
	led_num++;
	return 0;
}
int  Manager::get_LED_Num()
{
	return led_num;
}
LED* Manager::getLED(const int num)
{
	//check if not valid,return null pointer:
	if(num>led_num)
		return NULL;
	return led_table[num].pLED;
}
LED* Manager::getLED(const char *name)
{
	for(int i=0;i<LED_NUM;i++)
	{
			//check if valid,return pointer:
			if(0==strcmp(name,led_table[i].name))
			{
				return led_table[i].pLED;
			}
	}
	//check if not valid,return null pointer:
	return NULL;
}


//Manager::UART part
int Manager::Register_UART(const char *name,IUART *pUart)
{
	strcpy(uart_table[uart_num].name,name);
	uart_table[uart_num].num=uart_num;
	uart_table[uart_num].pUart=pUart;
	uart_num++;
	return 0;
}
int Manager::get_UART_Num()
{
	return uart_num;
}
IUART *Manager::getUART(const int num)
{
	if(num>uart_num)
		return NULL;
	return uart_table[num].pUart;
}
IUART *Manager::getUART(const char *name)
{
	for(int i=0;i<UART_NUM;i++)
	{
			//check if valid,return pointer:
			if(0==strcmp(name,uart_table[i].name))
			{
				return uart_table[i].pUart;
			}
	}
	//check if not valid,return null pointer:
	return NULL;
}
	
//Manager::Timer part:
int Manager::Register_Timer(const char *name,ITimer *pTimer)
{
	strcpy(timer_table[timer_num].name,name);
	timer_table[timer_num].num=timer_num;
	timer_table[timer_num].pTimer=pTimer;
	timer_num++;
	return 0;
}
int Manager::get_Timer_Num()
{
	return timer_num;
}
ITimer *Manager::getTimer(const int num)
{
	if(num>timer_num)
		return NULL;
	return timer_table[num].pTimer;
}
ITimer *Manager::getTimer(const char *name)
{
	for(int i=0;i<TIMER_NUM;i++)
	{
			//check if valid,return pointer:
			if(0==strcmp(name,timer_table[i].name))
			{
				return timer_table[i].pTimer;
			}
	}
	//check if not valid,return null pointer:
	return NULL;
}
int Manager::Register_BatteryVoltage(const char *name,IBatteryVoltage *pIBatteryVoltage)
{
	strcpy(batteryvoltage_table[batteryvoltage_num].name,name);
	batteryvoltage_table[batteryvoltage_num].pIBatteryVoltage=pIBatteryVoltage;
	batteryvoltage_num++;
	return 0;
}
IBatteryVoltage *Manager::getBatteryVoltage(const char *name)
{
	for(int i=0;i<BATTERYVOLTAGE_NUM;i++)
	{
			//check if valid,return pointer:
			if(0==strcmp(name,batteryvoltage_table[i].name))
			{
				return batteryvoltage_table[i].pIBatteryVoltage;
			}
	}
	//check if not valid,return null pointer:
	return NULL;
}

Manager manager;