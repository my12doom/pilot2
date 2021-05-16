#include "si5351.h"
#include <math.h>

int test_si5351(HAL::II2C *i2c)
{
	si5351 si;
    si.set_ref_freq(26.0);
	si.set_i2c(i2c);
	si.set_pll(0, 770, si5351::XTAL);
    si.set_output_freq(0, 0, 4);
    si.set_output(0, false, false);
}

si5351::si5351(float crystal)
{
	ref = crystal;
}

si5351::~si5351()
{

}

int si5351::set_i2c(HAL::II2C *i2c, uint8_t address)
{
	this->i2c = i2c;
	this->address = address;

    int o = i2c->write_reg(address, 187, 0xd0);
    i2c->write_reg(address, 3, 0xff);
    i2c->write_reg(address, 9, 0);
	for(int i=16; i<=23; i++)
		i2c->write_reg(address, i, 0x80);	
		
	return o;
}

int si5351::set_ref_freq(float freq)
{
    ref = freq;

    return 0;
}

int si5351::set_pll(int id, double freq, int freq_src, int input_divider)
{
    if (freq < 600 || freq > 900 || id <0 || id > 1)
		return -1;

    uint8_t reg15;
    if (!i2c || i2c->read_reg(address, 15, &reg15) < 0)
        return -1;

    uint8_t div_setting = 0;
    while(input_divider>1)
    {
        div_setting ++;
        input_divider >>= 1;
    }

    // clear input divider & PLL_SRC bits
    reg15 &= id == 0 ? 0x3B : 0x37;
    
    // freq_src
    reg15 |= freq_src << (id+2);
    reg15 |= div_setting << 6;

    if (!i2c || i2c->write_reg(address, 15, reg15) < 0)
        return -2;

    float abc = freq/ref;
	
	int a = floor(abc);	
	int c = 1<<18;
	int b = (abc-a)*c;	
	
	int p1 = 128*a+(128*b/c) -512;
	int p2 = 128*b-c*(128*b/c);
	int p3 = c;	
	
	uint8_t regs[8] =
	{
		(p3 >> 8) & 0xff,
		p3 & 0xff,
		(p1 >> 16) & 0x3,
		(p1 >> 8) & 0xff,
		p1 & 0xff,
		((p3 >> 12)&0xf0) | ((p2>>16)&0x0f),
		(p2 >> 8) & 0xff,
		p2 & 0xff,		
	};
	
	int reg_start = id ? 34 : 26;
	int o = i2c->write_regs(0xc0, reg_start, regs, 8);

    // reset both PLL
    i2c->write_reg(0xc0, 177, 0xAC);

    vco_freq[id] = freq;

	return o;
}

int si5351::set_output_freq(int id, int freq_src, double divider)
{
	if (id < 0 || id > 7)
		return -1;
	
	if (divider > 900 * 128)
		return -2;
	
	int R_DIV = 0;
	while (divider > 899.99)
	{
		R_DIV ++;
		divider *= 0.5f;
	}
		
	
	int a = floor(divider);	
	int c = 1<<16;
	int b = (divider-a)*c;	
	
	int p1 = 128*a+(128*b/c) -512;
	int p2 = 128*b-c*(128*b/c);
	int p3 = c;

    bool highfreq = vco_freq[freq_src]/divider > 150;
	
	if (highfreq)
	{
		R_DIV = p1 = p2 = 0;
		p3 = 1;
	}
	
	int o = -1;
	
	if (id == 6 || id == 7)
	{
		i2c->write_reg(0xc0, 0x5a + id - 6, a);
		
		uint8_t reg92 = 0;
		i2c->read_reg(0xc0, 0x5c, &reg92);
		
		reg92 &= id == 6 ? 0xf0 : 0x0f;
		reg92 |= R_DIV << (id == 6 ? 0 : 4);
		o = i2c->write_reg(0xc0, 0x5c, reg92);
	}
	else
	{
	
		uint8_t regs[8] =
		{
			(p3 >> 8) & 0xff,
			p3 & 0xff,
			(R_DIV << 4) | ((p1 >> 16) & 0x3) | (highfreq ? 0xC : 0),
			(p1 >> 8) & 0xff,
			p1 & 0xff,
			((p3 >> 12)&0xf0) | ((p2>>16)&0x0f),
			(p2 >> 8) & 0xff,
			p2 & 0xff,		
		};
		
		int reg_start = 42 + id * 8;
		o = i2c->write_regs(0xc0, reg_start, regs, 8);
	}

	uint8_t reg16;
	i2c->read_reg(0xc0, 16 + id, &reg16);
	reg16 &= 0x9f;
	reg16 |= (b == 0 && id < 6) ? 0x40 : 0;
    reg16 |= freq_src << 5;
	i2c->write_reg(0xc0, 16 + id, reg16);
	
	return o;	
}

int si5351::set_output(int id, bool power_down, bool invert, output_type output_type, drive_current drive_current)
{
    if (id < 0 || id > 7)
        return -1;
    
    uint8_t reg3;
    i2c->read_reg(address, 3, &reg3);
    reg3 &= ~(1<<id);
    if (power_down)
        reg3 |= 1<<id;
	i2c->write_reg(address, 3, reg3);

	uint8_t reg = 16 + id;
    uint8_t v;
    i2c->read_reg(address, reg, &v);

    v &= 0x60;
    v |= power_down ? 0x80 : 0;
    v |= invert ? 0x10 : 0;
    v |= output_type << 2;
    v |= drive_current;

    return i2c->write_reg(address, reg, v);
}
