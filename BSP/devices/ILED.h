#pragma once
#include "F4GPIO.h"
using namespace STM32F4;
namespace STM32F4
{
	class ILED
	{
	public:
		virtual void on()=0;
		virtual void off()=0;
		virtual void toggle()=0;
	};
}
