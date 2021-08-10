#include "MAX2871.h"
#include <HAL/Interface/ISysTimer.h>

MAX2871::MAX2871()
:reg6(0)
,reg5(0x00400005)
,reg4(0x6180B23C)
,reg3(0x0000000B)
,reg2(0x00004042)
,reg1(0x2000FFF9)
,reg0(0x007D0000)
{

}

MAX2871::~MAX2871()
{

}

int MAX2871::init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *rf_en /*= NULL*/, HAL::IGPIO *LD /*= NULL*/)
{
	this->spi = spi;
	this->cs = cs;
	this->LD = LD;
	this->rf_en = rf_en;

	if (rf_en)
		rf_en->set_mode(HAL::MODE_OUT_PushPull);
	if (LD)
		LD->set_mode(HAL::MODE_IN);

	if (!cs)
		return -1;

	cs->set_mode(HAL::MODE_OUT_PushPull);
	spi->set_mode(0, 0);
	spi->set_speed(10000000);

	// configure MUX to 0b1100 (MISO)
	reg2.MUX = 4;
	reg5.MUX3 = 1;
	reg2.MUX = 0;
	reg5.MUX3 = 0;

	write_reg(6, reg6);
	write_reg(5, reg5);
	systimer->delayms(20);
	reg4.RF_EN = reg4.AUX_RF_EN = 0;
	write_reg(4, reg4);
	write_reg(3, reg3);
	write_reg(2, reg2);
	write_reg(1, reg1);
	write_reg(0, reg0);

	// 
	reg2.LDP = 1;
	reg2.CP_CURRENT = 15;

	return 0;
}

int MAX2871::set_ref(uint32_t freq, bool doubler, bool divider, int R)
{
	reg2.REF_DOUBLE = doubler;
	reg2.R = R;
	reg2.RDIV2 = divider;

	pfd_freq = freq;
	if (doubler)
		pfd_freq *= 2;
	if (divider)
		pfd_freq /= 2;

	pfd_freq /= R;

	write_reg(2, reg2);

	// max allowed pfd freqency is 140Mhz, might work a little higher
	return pfd_freq <= 140000000 ? 0 : -1;
}

int MAX2871::set_output(bool rfen, bool out_main, bool out_aux)
{
	if (rf_en)
		rf_en->write(rfen);

	reg4.RF_EN = out_main;
	reg4.AUX_RF_EN = out_aux;
	reg4.POWER = 0;

	write_reg(4, reg4);

	return 0;
}

int MAX2871::set_freq(uint64_t freq)
{
	// allowed vco range: 3G to 6G
	uint64_t vco_freq = freq;
	reg4.RF_DIV = 0;
	while (vco_freq<3000000000)
	{
		reg4.RF_DIV ++;
		vco_freq *= 2;
	}

	reg0.N_INT = vco_freq/pfd_freq;
	reg0.N_FRAC = (vco_freq - (uint64_t)reg0.N_INT * pfd_freq) * 0xfff / pfd_freq;

	write_reg(4, reg4);
	write_reg(0, reg0);

	return 0;
}

bool MAX2871::is_locked()
{
	if (LD)
		return LD->read();
	
	return false;
}

void MAX2871::write_reg(int address, uint32_t data)
{
	data &= 0xfffffff8;
	data |= address & 0x7;

	uint8_t buf[4] = 
	{
		uint8_t(data >> 24),
		uint8_t(data >> 16),
		uint8_t(data >> 8),
		uint8_t(data >> 0),
	};

	cs->write(0);
	spi->txrx2(buf, buf, 4);
	cs->write(1);
}
