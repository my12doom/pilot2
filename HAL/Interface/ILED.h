#pragma once
#include <HAL/Interface/IGPIO.h>
namespace devices
{
	class ILED
	{
	public:
		virtual void on()=0;
		virtual void off()=0;
		virtual void toggle()=0;
	};

	class GPIOLED : public ILED
	{
	public:
		GPIOLED(HAL::IGPIO *gpio, bool pushpull=false){init(gpio, pushpull);}
		GPIOLED(){}
		~GPIOLED(){}
		void init(HAL::IGPIO *gpio, bool pushpull=false){gpio->set_mode(pushpull?HAL::MODE_OUT_PushPull:HAL::MODE_OUT_OpenDrain);this->gpio=gpio;this->pushpull=pushpull;}
		virtual void on(){gpio->write(false^pushpull);}
		virtual void off(){gpio->write(true^pushpull);}
		virtual void toggle(){gpio->toggle();}
	private:
		HAL::IGPIO *gpio;
		bool pushpull;
	};
}
