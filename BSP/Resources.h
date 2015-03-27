#pragma once
#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/STM32F4/F4Interfaces.h>
#include <BSP\devices\ILED.h>
#include <BSP\boards\dev_v1\LED.h>
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
	typedef struct a{
		char name[6];
		uint8_t num;
		LED *pLED; 
	}LED_table;
	private:
		LED_table led_table[LED_NUM];
		int led_num;
	public :
		virtual int  Register_LED(const char *name,LED *pLED);
		virtual int  get_LED_Num();
		virtual LED* getLED(const int num);
		virtual LED* getLED(const char *name);
	
	//Baro Manager:
	private:
	public :
		
};
Manager manager();
/*

class manager
{
	register_adc(IADC *adc);
	int get_adc_count();
	IADC *adc get_adc(int index);
};

manager mananer;
*/