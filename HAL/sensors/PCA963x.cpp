#include "PCA963x.h"

namespace devices
{

int PCA963x::read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;

	return i2c->read_regs(address, reg, (uint8_t*)out, count);
}

int PCA963x::write_reg_core(uint8_t reg, uint8_t data)
{
	return i2c->write_reg(address, reg, data);
}

int PCA963x::write_reg(uint8_t reg, uint8_t data)
{
	uint8_t read;
	for(int i=0; i<10; i++)
	{
		read = data - 1;
		write_reg_core(reg, data);
		read_reg(reg, &read, 1);

		if (read == data)
			return 0;
		//else
		//	printf("reg(%02x) error %02x/%02x\n", reg, read, data);
		
		systimer->delayms(10);
	}

	return -1;
}

PCA963x::PCA963x()
{
}

// ILED
void PCA963x::on()
{
}
void PCA963x::off()
{
}
void PCA963x::toggle()
{
}

// IRGBLED
int PCA963x::write(float R, float G, float B)
{
}


PCA963x a;
}