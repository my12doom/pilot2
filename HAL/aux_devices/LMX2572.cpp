#include "LMX2572.h"
#include "LMX2572_default.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#define VCO_CAL_THRESHOLD 25000000

LMX2572::LMX2572()
{

}
LMX2572::~LMX2572()
{

}
int LMX2572::init(HAL::ISPI *spi, HAL::IGPIO *cs)
{
	this->spi = spi;
	this->cs = cs;
	last_vco_sel_freq = 0;

	if (!cs || !spi)
		return -1;

	cs->set_mode(HAL::MODE_OUT_PushPull);
	cs->write(1);
	spi->set_speed(20000000);
	spi->set_mode(0,0);

	memcpy(regs, LMX2572_default_regs, sizeof(regs));
	for(int i=0; i<=126; i++)
		write_reg(i, regs[i]);

	return 0;
}

int LMX2572::set_ref(uint32_t ref_freq, bool doubler, int pre_R, int multiplier, int R, bool diff /*= false*/)
{
	// note:multiplier *8-31 is allowed but not recommended
	// also use doubler instead of multiplier 2 

	// 
	if(doubler)
		ref_freq *= 2;

	pre_R &= 0xfff;		// 12bit pre-R;
	multiplier &= 0x1f;	// 5bit multiplier
	R &= 0xff;			// 8bit R
	
	pfd_freq = ref_freq * multiplier / pre_R / R;

	regs[11] = 0xB008 | (R<<4);
	regs[10] = 0x1078 | (multiplier<<7);
	regs[9] = 0x4;
	if (doubler)
		regs[9] |= 0x1000;
	if (pfd_freq > 100000000)
		regs[9] |= 0x4000;

	for(int i=9; i<=11; i++)
		write_reg(i, regs[i]);

	regs[5] = (regs[5] &0xEFFF) | (diff ? 0 : 0x1000);
	write_reg(5, regs[5]);

	return 0;
}

int LMX2572::set_output(bool enableA, int powerA, bool enableB /*= false*/, int powerB /*= 63*/)
{
	powerA &= 0x3f;
	regs[44] = 0x22 | (powerA << 8);
	if (!enableA)
		regs[44] |= 0x40;
	if (!enableB)
		regs[44] |= 0x80;
	write_reg(44, regs[44]);

	regs[45] = (regs[45] & 0xFFC0) | (powerB&0x3f);
	write_reg(45, regs[45]);

	return 0;
}

int LMX2572::set_freq(uint64_t freq)
{
	uint32_t denum = 0xffffff;
	// allowed vco range: 3.2G to 6.4G
	uint64_t vco_freq = freq;
	int div = 0;
	while (vco_freq<3200000000)
	{
		div ++;
		vco_freq *= 2;
	}

	if (div > 8)
		return -1;

	int N = vco_freq/pfd_freq;
	int FRAC = (vco_freq - N * (uint64_t)pfd_freq) * denum / pfd_freq;



	regs[34] = ((N>>16)&0x7) | 0x10;
	regs[36] = N;
	regs[38] = denum>>16;
	regs[39] = denum;
	regs[42] = FRAC>>16;
	regs[43] = FRAC;
	regs[45] = ((div == 0) ? 0x0800 : 0) | (regs[45]&0xE7FF);	// OUT_A MUX
	regs[46] = (div == 0) ? 0x07F1 : 0x07F0;					// OUT_B MUX

	// strange divider table of R75
	int divider_tbl[9] = {0, 0, 1, 3, 5, 7, 9, 12, 14};
	regs[75] = 0x0800 | (divider_tbl[div]<<6);
	
	int freq_delta = last_vco_sel_freq > freq ? last_vco_sel_freq - freq : freq - last_vco_sel_freq;
	if (freq_delta > VCO_CAL_THRESHOLD)
	{
		last_vco_sel_freq = freq;
		regs[78] &= ~0x200;
	}
	else
	{
		//regs[78] |= 0x200;
	}


	// Partial Assist
	int mhz = vco_freq / 1000000;
	int assist_tbl[6][6] = 
	{
		{3200, 3650, 131, 19, 138, 137},
		{3650, 4200, 143, 25, 162, 142},
		{4200, 4650, 135, 34, 126, 114},
		{4650, 5200, 136, 25, 195, 172},
		{5200, 5750, 133, 20, 190, 163},
		{5750, 7000, 151, 27, 256, 204},
	};

	int select = -1;
	for(int i=0; i<6; i++)
	{
		if (mhz >= assist_tbl[i][0] && mhz <= assist_tbl[i][1])
			select = i;
	}

	if (select == 3 && mhz < 4700)
		select = 2;

	if (select == 3 && mhz > 5100)
		select = 4;
	
	int fmin = assist_tbl[select][0];
	int fmax = assist_tbl[select][1];
	int cmin = assist_tbl[select][2];
	int cmax = assist_tbl[select][3];
	int amin = assist_tbl[select][4];
	int amax = assist_tbl[select][5];

	int C = floor(0.5f + cmin - float(mhz-fmin) * (cmin-cmax) / (fmax-fmin));
	int A = floor(0.5f + amin - float(mhz-fmin) * (amin-amax) / (fmax-fmin));
	int vco = select + 1;

	regs[78] = 0x000 | (C<<1);
	regs[20] = 0x448 | (vco << 11);
	regs[17] = A;
	regs[8] = 0x6000;
	write_reg(20, regs[20]);
	write_reg(17, regs[17]);
	write_reg(8, regs[8]);

	//printf("freq%dMhz, vco%d A%d C%d\n", mhz, vco, A, C);

	


	write_reg(78, regs[78]);
	write_reg(75, regs[75]);
	write_reg(46, regs[46]);
	write_reg(45, regs[45]);
	write_reg(39, regs[39]);
	write_reg(38, regs[38]);
	write_reg(43, regs[43]);
	write_reg(42, regs[42]);
	write_reg(36, regs[36]);
	write_reg(34, regs[34]);		// write N last
	write_reg(0, regs[0]);			// FCAL_EN

	return 0;
}

bool LMX2572::is_locked()
{
	uint16_t reg110 = read_reg(110);
	return (reg110>>9) == 2;
}
void LMX2572::write_reg(uint8_t address, uint16_t data)
{
	cs->write(0);
	spi->txrx(address&0x7f);
	spi->txrx((data>>8)&0xff);
	spi->txrx(data&0xff);
	cs->write(1);
}
uint16_t LMX2572::read_reg(uint8_t address)
{
	uint16_t value;
	cs->write(0);
	spi->txrx((address&0x7f) | 0x80);
	value = spi->txrx(0) << 8;
	value |= spi->txrx(0);
	cs->write(1);

	return value;
}
