#pragma once
#include <stdint.h>
#include <HAL/Interface/Interfaces.h>

namespace devices
{

class IBatteryVoltage
{
	public :
		virtual float read()=0;
};

class ADCBatteryVoltage :public IBatteryVoltage
{
	private:
		float scale;
		IADC * adc;
	public :
		ADCBatteryVoltage(){}
		ADCBatteryVoltage(IADC * adc,float scale){init(adc, scale);}
		void init(IADC * adc,float scale){this->scale = scale; this->adc = adc;}
		~ADCBatteryVoltage(){};
		virtual float read(){return adc->read()*scale;}
};

}