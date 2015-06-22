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
		GPIOLED(HAL::IGPIO *gpio){init(gpio);}
		GPIOLED(){}
		~GPIOLED(){}
		void init(HAL::IGPIO *gpio){gpio->set_mode(HAL::MODE_OUT_OpenDrain);this->gpio=gpio;}
		virtual void on(){gpio->write(false);}
		virtual void off(){gpio->write(true);}
		virtual void toggle(){gpio->toggle();}
	private:
		HAL::IGPIO *gpio;
	};
}
