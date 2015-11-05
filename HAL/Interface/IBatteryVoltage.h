#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
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
	HAL::IADC * adc;
	float ref_voltage;
	HAL::IADC * ref;
public :
	ADCBatteryVoltage(){}
	ADCBatteryVoltage(HAL::IADC * adc,float scale, HAL::IADC * ref = NULL,float ref_voltage = 0){init(adc, scale, ref, ref_voltage);}
	void init(HAL::IADC * adc,float scale, HAL::IADC * ref = NULL,float ref_voltage = 0){this->scale = scale; this->adc = adc; this->ref = ref; this->ref_voltage = ref_voltage;}
	~ADCBatteryVoltage(){};
	virtual float read()
	{
		float scale_ref = 1.0f;
		if (ref)
			scale_ref = ref_voltage / ref->read();
		
		return adc->read()*scale*scale_ref;
	}
};

class ADCReference : public IBatteryVoltage
{
public:
	ADCReference(){}
	ADCReference(HAL::IADC * ref, float ref_voltage, float full_scale):_ref(ref),_full_scale(full_scale),_ref_voltage(ref_voltage){}
	~ADCReference(){};
	virtual float read()
	{
		return _full_scale / _ref->read() * _ref_voltage;
	}

private:
	HAL::IADC * _ref;
	float _full_scale;
	float _ref_voltage;
};


class differential_monitor : public IBatteryVoltage
{
public:
	differential_monitor(IBatteryVoltage* vcc, IBatteryVoltage *adc, float scale)
	:_vcc(vcc)
	,_adc(adc)
	,_scale(scale)
	{
	}
	~differential_monitor(){}
	virtual float read()
	{
		return fabs(_vcc->read() - _adc->read()) * _scale;
	}

	IBatteryVoltage* _vcc;
	IBatteryVoltage* _adc;
	float _scale;
};

}