#pragma once

#include <stdint.h>

class RGBLED
{
public:
	RGBLED();
	~RGBLED(){}
	virtual int write(float R, float G, float B);
};
