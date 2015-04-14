#include <BSP\boards\dev_v1\LED.h>
using namespace STM32F4;
namespace BSP{
	LED::LED(F4GPIO * f4gpio)
	{
		this->f4gpio=f4gpio;
	}
	void LED::on()
	{
		f4gpio->write(0);
	}
	void LED::off()
	{
		f4gpio->write(1);
	}
	void LED::toggle()
	{
		f4gpio->toggle();
	}
}