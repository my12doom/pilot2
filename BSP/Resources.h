#pragma once
#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/STM32F4/F4Interfaces.h>
#include <BSP/devices/ILED.h>
#include <BSP/boards/dev_v1/LED.h>
#include <BSP/boards/dev_v1/BatteryVoltage.h>
using namespace BSP;
using namespace STM32F4;
class Manager
{
	//Manager construct:
	public :
		Manager(){};
		~Manager(){};
	
	//LED Manager:
	#define LED_NUM 5
	typedef struct{
		char name[10];//name 
		uint8_t num; //num
		LED *pLED;   //pointer
	}LED_table;
	private:
		LED_table led_table[LED_NUM];
		int led_num;
	public :
		virtual int  Register_LED(const char *name,LED *pLED);
		virtual int  get_LED_Num();
		virtual LED* getLED(const int num);
		virtual LED* getLED(const char *name);
	
	//Uart Manager:
	#define UART_NUM 4
	typedef struct{
		char name[10];
		uint8_t num;
		IUART *pUart; 
	}UART_table;
	private:
		UART_table uart_table[UART_NUM];
		int uart_num;
	public :
		virtual int  Register_UART(const char *name,IUART *pUart);
		virtual int  get_UART_Num();
		virtual IUART *getUART(const int num);
		virtual IUART *getUART(const char *name);
	
	//Timer Manager:
	#define TIMER_NUM 4
	typedef struct{
		char name[10];
		uint8_t num;
		ITimer *pTimer; 
	}Timer_table;
	private:
		Timer_table timer_table[TIMER_NUM];
		int timer_num;
	public :
		virtual int  Register_Timer(const char *name,ITimer *pTimer);
		virtual int  get_Timer_Num();
		virtual ITimer *getTimer(const int num);
		virtual ITimer *getTimer(const char *name);
	
	
	//BatteyVotage Manager:
	#define BATTERYVOLTAGE_NUM 1
	private:
		typedef struct{
		char name[10];
		uint8_t num;
		IBatteryVoltage *pIBatteryVoltage; 
	}BatteryVoltage_table;
		BatteryVoltage_table batteryvoltage_table[BATTERYVOLTAGE_NUM];
		int batteryvoltage_num;
	public :
		virtual int Register_BatteryVoltage(const char *name,IBatteryVoltage *pIBatteryVoltage);
		virtual IBatteryVoltage *getBatteryVoltage(const char *name);
	
	
};
//Declear manager as global:
extern Manager manager;