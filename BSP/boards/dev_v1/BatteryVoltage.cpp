#include <BSP\boards\dev_v1\BatteryVoltage.h>
using namespace STM32F4;
namespace BSP{
BatteryVoltage::BatteryVoltage(F4ADC * f4adc,float scale)
{
	this->f4adc=f4adc;
	this->scale=scale;
}
float BatteryVoltage::read()
{
	return f4adc->read()*scale;
}
}