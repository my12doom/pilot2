#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>

struct ADF4351_REG0
{
	unsigned ADDR:3;
	unsigned N_FRAC:12;
	unsigned N_INT:16;
	unsigned reserved:1;

	ADF4351_REG0(uint32_t v)
	{
		*this = *(ADF4351_REG0*)&v;
	}

	ADF4351_REG0& operator =(uint32_t v)
    {
		*this = *(ADF4351_REG0*)&v;
        return *this;
    }
    operator uint32_t()
	{
		return *(uint32_t*)this;
	}
};


struct ADF4351_REG1
{
	unsigned ADDR:3;
	unsigned MOD:12;
	unsigned PHASE:12;
	unsigned PRESCALER:1;
	unsigned PHASE_ADJUST:1;
	unsigned reserved:3;

	ADF4351_REG1(uint32_t v)
	{
		*this = *(ADF4351_REG1*)&v;
	}

	ADF4351_REG1& operator =(uint32_t v)
    {
		*this = *(ADF4351_REG1*)&v;
        return *this;
    }
    operator uint32_t()
	{
		return *(uint32_t*)this;
	}
};

struct ADF4351_REG2
{
	unsigned ADDR:3;
	unsigned COUNTER_RESET:1;
	unsigned CP_3STATE:1;
	unsigned POWERDOWN:1;
	unsigned PD_POLARITY:1;
	unsigned LDP:1;
	unsigned LDF:1;
	unsigned CHARGE_PUMP_CURRENT:4;
	unsigned DOUBLE_BUFFER:1;
	unsigned R:10;
	unsigned RDIV2:1;
	unsigned REF_DOUBLER:1;
	unsigned MUX_OUT:3;
	unsigned LOW_NOISE:2;
	unsigned reserved:1;

	ADF4351_REG2(uint32_t v)
	{
		*this = *(ADF4351_REG2*)&v;
	}

	ADF4351_REG2& operator =(uint32_t v)
    {
		*this = *(ADF4351_REG2*)&v;
        return *this;
    }
    operator uint32_t()
	{
		return *(uint32_t*)this;
	}
};

struct ADF4351_REG3
{
	unsigned ADDR:3;
	unsigned CLOCK_DIVIDER:12;
	unsigned CLOCK_DIV_MODE:2;
	unsigned reserved:1;
	unsigned CSR:1;
	unsigned reserved2:2;
	unsigned CHARGE_CANCEL:1;
	unsigned ABP:1;
	unsigned BAND_SELECT_MODE:1;
	unsigned reserved3:8;

	ADF4351_REG3(uint32_t v)
	{
		*this = *(ADF4351_REG3*)&v;
	}

	ADF4351_REG3& operator =(uint32_t v)
    {
		*this = *(ADF4351_REG3*)&v;
        return *this;
    }
    operator uint32_t()
	{
		return *(uint32_t*)this;
	}
};

struct ADF4351_REG4
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
	unsigned reserved:8;

	ADF4351_REG4(uint32_t v)
	{
		*this = *(ADF4351_REG4*)&v;
	}

	ADF4351_REG4& operator =(uint32_t v)
    {
		*this = *(ADF4351_REG4*)&v;
        return *this;
    }
    operator uint32_t()
	{
		return *(uint32_t*)this;
	}
};

struct ADF4351_REG5
{
	unsigned ADDR:3;
	unsigned reserved:19;
	unsigned LD_MODE:2;
	unsigned reserved2:8;

	ADF4351_REG5(uint32_t v)
	{
		*this = *(ADF4351_REG5*)&v;
	}

	ADF4351_REG5& operator =(uint32_t v)
    {
		*this = *(ADF4351_REG5*)&v;
        return *this;
    }
    operator uint32_t()
	{
		return *(uint32_t*)this;
	}
};

class ADF4351
{
public:
	ADF4351();
	~ADF4351();
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

	ADF4351_REG5 reg5;
	ADF4351_REG4 reg4;
	ADF4351_REG3 reg3;
	ADF4351_REG2 reg2;
	ADF4351_REG1 reg1;
	ADF4351_REG0 reg0;

	uint32_t pfd_freq;
};
