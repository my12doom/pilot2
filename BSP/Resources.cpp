#include <BSP\Resources.h>
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
	return 1;
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