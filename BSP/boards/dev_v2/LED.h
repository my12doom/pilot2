#pragma once
#include <BSP\devices\ILED.h>
using namespace STM32F4;
namespace BSP{
	class LED :public ILED
	{
	private:
		F4GPIO * f4gpio;
	public:
		LED(F4GPIO * f4gpio);
	    ~LED(){};
		virtual void on();
		virtual void off();
		virtual void toggle();
	};
}