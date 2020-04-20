#include "ADF4351.h"

ADF4351::ADF4351()
:reg5(0x00580005U)
,reg4(0x008c803cU)
,reg3(0x000004b3U)
,reg2(0x00010e42U)
,reg1(0x00008029U)
,reg0(0x002c8008U)
{

}

ADF4351::~ADF4351()
{

}

int ADF4351::init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *rf_en /*= NULL*/, HAL::IGPIO *LD /*= NULL*/)
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

	reg1.MOD = 0xfff;
	reg2.LOW_NOISE = 3;
	reg2.R = 256;
	reg2.CHARGE_PUMP_CURRENT = 15;
	reg3.BAND_SELECT_MODE = 1;
	reg4.BAND_SELECT_DIV = 50;
	reg4.AUX_SELECT = 0;

	write_reg(5, reg5);
	write_reg(4, reg4);
	write_reg(3, reg3);
	write_reg(2, reg2);
	write_reg(1, reg1);
	write_reg(0, reg0);

	return 0;
}

int ADF4351::set_ref(uint32_t freq, bool doubler, bool divider, int R)
{
	reg2.REF_DOUBLER = doubler;
	reg2.R = R;
	reg2.RDIV2 = divider;

	pfd_freq = freq;
	if (doubler)
		pfd_freq *= 2;
	if (divider)
		pfd_freq /= 2;

	pfd_freq /= R;

	write_reg(2, reg2);

	// max allowed pfd freqency is 32Mhz, might work a little higher
	return pfd_freq <= 32000000 ? 0 : -1;
}

int ADF4351::set_output(bool rfen, bool out_main, bool out_aux)
{
	if (rf_en)
		rf_en->write(rfen);

	reg4.RF_EN = out_main || out_aux;		// chip BUG: this bit also disables aux output..
	reg4.AUX_RF_EN = out_aux;
	reg4.AUX_POWER = 3;
	reg4.AUX_SELECT = 0;

	write_reg(4, reg4);

	return 0;
}

int ADF4351::set_freq(uint64_t freq)
{
	// allowed vco range: 2.2G to 4.4G
	uint64_t vco_freq = freq;
	reg4.RF_DIV = 0;
	while (vco_freq<2200000000)
	{
		reg4.RF_DIV ++;
		vco_freq *= 2;
	}

	reg0.N_INT = vco_freq/pfd_freq;
	reg0.N_FRAC = (vco_freq - reg0.N_INT * pfd_freq) * 0xfff / pfd_freq;

	write_reg(4, reg4);
	write_reg(0, reg0);

	return 0;
}

bool ADF4351::is_locked()
{
	if (LD)
		return LD->read();
	
	return false;
}

void ADF4351::write_reg(int address, uint32_t data)
{
	data &= 0xfffffff8;
	data |= address & 0x7;

	uint8_t buf[4] = 
	{
		data >> 24,
		data >> 16,
		data >> 8,
		data >> 0,
	};

	cs->write(0);
	spi->txrx2(buf, buf, 4);
	cs->write(1);
}
