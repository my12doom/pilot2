#pragma once
#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/STM32F4/F4Interfaces.h>
#include <BSP/devices/IBatteryVoltage.h>
#include "stm32F4xx_adc.h"
using namespace HAL;
using namespace STM32F4;
namespace BSP{
	class BatteryVoltage :public IBatteryVoltage
	{
		private:
			int scale;
			F4ADC * f4adc;
		public :
			BatteryVoltage(F4ADC * f4adc,float scale);
			~BatteryVoltage(){};
			virtual float read();
	};
}