#pragma once
#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/STM32F4/F4Interfaces.h>
#include "stm32F4xx_adc.h"
using namespace HAL;
using namespace STM32F4;
class IBatteryVoltage
{
	public :
		virtual float read()=0;
};