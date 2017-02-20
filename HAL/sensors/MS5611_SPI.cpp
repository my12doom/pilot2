#include "MS5611_SPI.h"
#include <string.h>

using namespace HAL;

// registers of the device
#define MS561101BA_ADDR_CSB_LOW   0xEC
#define MS5611Address (0x77<<1)
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2	// by adding ints from 0 to 6 we can read all the prom configuration values. 
										// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.
#define EXTRA_PRECISION 5 // trick to add more precision to the pressure and temp readings

#define SAMPLEING_TIME 10000 // 10000us, 10ms

namespace sensors
{

MS5611_SPI::MS5611_SPI()
{
	
	OSR = MS561101BA_OSR_4096;
	temperature = 0;
	pressure = 0;
	new_temperature = 0;
	last_temperature_time = 0;
	last_pressure_time = 0;
	rawTemperature = 0;
	rawPressure = 0;
	DeltaTemp = 0;
}

MS5611_SPI::~MS5611_SPI()
{
}


int MS5611_SPI::read_regs(uint8_t start_reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	uint8_t tx_buf[16] = {0};
	uint8_t rx_buf[16];
	tx_buf[0] = start_reg;
	CS->write(false);

	spi->txrx2(tx_buf, rx_buf, count+1);
	memcpy(out, rx_buf+1, count);

	CS->write(true);

	return 0;
}

int MS5611_SPI::write_reg(uint8_t reg)
{
	CS->write(false);

	spi->txrx(reg);

	CS->write(true);

	return 0;	
}

int MS5611_SPI::init(ISPI *spi, IGPIO *CS)
{
	this->spi = spi;
	this->CS = CS;
	CS->set_mode(MODE_OUT_PushPull);
	CS->write(true);
	spi->set_mode(1, 1);
	spi->set_speed(10000000);	// MS5611 SPI can handle 20mhz max

	uint8_t tmp[3];
	int i;
	
	write_reg(MS561101BA_RESET);	
	systimer->delayms(10);
	for(i=0; i<6; i++)
	{
		read_regs(MS561101BA_PROM_BASE_ADDR+i*2, tmp, 2);
		refdata[i] = (tmp[0] << 8) + tmp[1];
	}
	
	// Temperature
	write_reg(MS561101BA_D2 + OSR);
	systimer->delayms(10);
	read_regs(0x00, tmp, 3);
	
	rawTemperature = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
	DeltaTemp = rawTemperature - (((int32_t)refdata[4]) << 8);
	temperature = ((1<<EXTRA_PRECISION)*2000l + ((DeltaTemp * refdata[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION));
		
	// Pressure
	write_reg(MS561101BA_D1 + OSR);
	systimer->delayms(10);
	read_regs(0x00, tmp, 3);
	
	rawPressure = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
	off  = (((int64_t)refdata[1]) << 16) + ((refdata[3] * DeltaTemp) >> 7);
	sens = (((int64_t)refdata[0]) << 15) + ((refdata[2] * DeltaTemp) >> 8);
	pressure = ((((rawPressure * sens) >> 21) - off) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION));	
	
	return healthy() ? 0 : -1;
}

int MS5611_SPI::read(int *data)
{
	int rtn = 1;
	uint8_t tmp[3];

	spi->set_mode(1,1);
	spi->set_speed(10000000);	// MS5611 SPI can handle 20mhz max

	do
	{
		if (new_temperature == 0 && last_temperature_time == 0 && last_pressure_time == 0)
		{
			if (write_reg(MS561101BA_D2 + OSR) < 0)
			{
				rtn = -1;
				break;
			}
			last_temperature_time = systimer->gettime();
		}
		
		if (systimer->gettime() - last_temperature_time >  SAMPLEING_TIME && new_temperature == 0)
		{
			if (read_regs(0x00, tmp, 3) < 0)
			{
				rtn = -1;
				break;
			}
		
			rawTemperature = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
			DeltaTemp = rawTemperature - (((int32_t)refdata[4]) << 8);
			new_temperature = ((1<<EXTRA_PRECISION)*2000l + ((DeltaTemp * refdata[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION));
			
			if (write_reg(MS561101BA_D1 + OSR) < 0)
			{
				rtn = -1;
				break;
			}

			last_pressure_time = systimer->gettime();
		}

		if (systimer->gettime() - last_pressure_time >  SAMPLEING_TIME && last_pressure_time > 0)
		{
			if (read_regs(0x00, tmp, 3) <0)
			{
				rtn = -1;
				break;
			}
			
			rawPressure = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
			off  = (((int64_t)refdata[1]) << 16) + ((refdata[3] * DeltaTemp) >> 7);
			sens = (((int64_t)refdata[0]) << 15) + ((refdata[2] * DeltaTemp) >> 8);
			pressure = ((((rawPressure * sens) >> 21) - off) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION));
			temperature = new_temperature;

			new_temperature = 0;
			last_temperature_time = 0;
			last_pressure_time = 0;
			rtn = 0;
		}
	} while (0);
	
	data[0] = pressure;
	data[1] = temperature;
	return rtn;
}

int MS5611_SPI::read(devices::baro_data *out)
{
	int data[2];
	int res = read(data);
	
	out->pressure = data[0];
	out->temperature = data[1] / 100.0f;

	return res;
}

bool MS5611_SPI::healthy()
{
	uint8_t tmp[2];
	uint16_t data[8];
	for(int i=0; i<8; i++)
	{
		read_regs(0xA0+i*2, tmp, 2);
		data[i] = (tmp[0] << 8) + tmp[1];
	}
	
	return check_crc(data);
}

bool MS5611_SPI::check_crc(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;
	
	bool all_zero = true;
	for(int i=0; i<8; i++)
		if (n_prom[i] != 0)
			all_zero = false;
	
	if (all_zero)
		return false;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

}