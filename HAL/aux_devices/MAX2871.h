#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>

struct MAX2871_REG0
{
	unsigned ADDR:3;
	unsigned N_FRAC:12;
	unsigned N_INT:16;
	unsigned INT_MODE:1;

	MAX2871_REG0(int v)
	{
		*this = *(MAX2871_REG0*)&v;
	}

	MAX2871_REG0& operator =(int v)
    {
		*this = *(MAX2871_REG0*)&v;
        return *this;
    }
    operator int()
	{
		return *(int*)this;
	}
};


struct MAX2871_REG1
{
	unsigned ADDR:3;
	unsigned MOD:12;
	unsigned PHASE:12;
	unsigned CP_TEST:2;
	unsigned CP_LINEAR:2;
	unsigned reserved:1;

	MAX2871_REG1(int v)
	{
		*this = *(MAX2871_REG1*)&v;
	}

	MAX2871_REG1& operator =(int v)
    {
		*this = *(MAX2871_REG1*)&v;
        return *this;
    }
    operator int()
	{
		return *(int*)this;
	}
};

struct MAX2871_REG2
{
	unsigned ADDR:3;
	unsigned RST:1;
	unsigned CP_3STATE:1;
	unsigned SHDN:1;
	unsigned PDP:1;
	unsigned LDP:1;
	unsigned LDF:1;
	unsigned CP_CURRENT:4;
	unsigned DOUBLE_BUFFER:1;
	unsigned R:10;
	unsigned RDIV2:1;
	unsigned REF_DOUBLE:1;
	unsigned MUX:3;
	unsigned NoiseMode:2;
	unsigned LD_SPEED:1;

	MAX2871_REG2(int v)
	{
		*this = *(MAX2871_REG2*)&v;
	}

	MAX2871_REG2& operator =(int v)
    {
		*this = *(MAX2871_REG2*)&v;
        return *this;
    }
    operator int()
	{
		return *(int*)this;
	}
};

struct MAX2871_REG3
{
	unsigned ADDR:3;
	unsigned CLOCK_DIVIDER:12;
	unsigned CLOCK_DIV_MODE:2;
	unsigned MUTE_DELAY:1;
	unsigned CSM:1;
	unsigned reserved:5;
	unsigned VAS_TEMP:1;
	unsigned VAS_SHDN:1;
	unsigned VCO:6;

	MAX2871_REG3(int v)
	{
		*this = *(MAX2871_REG3*)&v;
	}

	MAX2871_REG3& operator =(int v)
    {
		*this = *(MAX2871_REG3*)&v;
        return *this;
    }
    operator int()
	{
		return *(int*)this;
	}
};

struct MAX2871_REG4
{
	unsigned ADDR:3;
	unsigned POWER:2;
	unsigned RF_EN:1;
	unsigned AUX_POWER:2;
	unsigned AUX_RF_EN:1;
	unsigned AUX_SELECT:1;
	unsigned MTLD:1;
	unsigned VCO_POWER_DOWN:1;
	unsigned BAND_SELECT_DIV:8;
	unsigned RF_DIV:3;
	unsigned FB_SELECT:1;
	unsigned BAND_SELECT_MSB:2;
	unsigned SDREF:1;
	unsigned SDDIV:1;
	unsigned SDLDO:1;
	unsigned reserved:3;

	MAX2871_REG4(int v)
	{
		*this = *(MAX2871_REG4*)&v;
	}

	MAX2871_REG4& operator =(int v)
    {
		*this = *(MAX2871_REG4*)&v;
        return *this;
    }
    operator int()
	{
		return *(int*)this;
	}
};

struct MAX2871_REG5
{
	unsigned ADDR:3;
	unsigned ADC_MODE:3;
	unsigned ADC_START:1;
	unsigned reserved:11;
	unsigned MUX3:1;
	unsigned reserved2:3;
	unsigned LD_OUT:2;
	unsigned F01:1;
	unsigned SHUTDOWN_PLL:1;
	unsigned reserved3:3;
	unsigned VAS_DELAY:2;
	unsigned reserved4:1;

	MAX2871_REG5(int v)
	{
		*this = *(MAX2871_REG5*)&v;
	}

	MAX2871_REG5& operator =(int v)
    {
		*this = *(MAX2871_REG5*)&v;
        return *this;
    }
    operator int()
	{
		return *(int*)this;
	}
};

struct MAX2871_REG6
{
	unsigned ADDR:3;
	unsigned VCO:6;
	unsigned VASA:1;
	unsigned reserved:5;
	unsigned ADCV:1;
	unsigned _ADC:7;
	unsigned POR:1;
	unsigned reserved2:4;
	unsigned die:4;

	MAX2871_REG6(int v)
	{
		*this = *(MAX2871_REG6*)&v;
	}

	MAX2871_REG6& operator =(int v)
    {
		*this = *(MAX2871_REG6*)&v;
        return *this;
    }
    operator int()
	{
		return *(int*)this;
	}
};

class MAX2871
{
public:
	MAX2871();
	~MAX2871();
	int init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *rf_en = NULL, HAL::IGPIO *LD = NULL);
	int set_ref(uint32_t freq, bool doubler, bool divider, int R);
	int set_output(bool rfen, bool out_main, bool out_aux);
	int set_freq(uint64_t freq);
	bool is_locked();
protected:
	void write_reg(int address, uint32_t data);
	HAL::ISPI *spi;
	HAL::IGPIO *cs;
	HAL::IGPIO *LD;
	HAL::IGPIO *rf_en;

	MAX2871_REG6 reg6;
	MAX2871_REG5 reg5;
	MAX2871_REG4 reg4;
	MAX2871_REG3 reg3;
	MAX2871_REG2 reg2;
	MAX2871_REG1 reg1;
	MAX2871_REG0 reg0;

	uint32_t pfd_freq;
};

