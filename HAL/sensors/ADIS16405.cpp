#include "ADIS16405.h"
#include <Protocol/common.h>

using namespace HAL;

enum adis16405_registers
{
	flash_count = 0,
	supply_measurement = 2,
	gyro_x = 4,
	gyro_y = 6,
	gyro_z = 8,
	accel_x = 0xa,
	accel_y = 0xc,
	accel_z = 0xe,
	mag_x = 0x10,
	mag_y = 0x12,
	mag_z = 0x14,
	temperature = 0x16,
	aux_adc = 0x18,
	gyro_bias_x = 0x1a,
	gyro_bias_y = 0x1c,
	gyro_bias_z = 0x1e,
	accel_bias_x = 0x20,
	accel_bias_y = 0x22,
	accel_bias_z = 0x24,
	mag_hard_iron_x = 0x26,
	mag_hard_iron_y = 0x28,
	mag_hard_iron_z = 0x2a,
	mag_soft_iron_x = 0x2c,
	mag_soft_iron_y = 0x2e,
	mag_soft_iron_z = 0x30,
	gpio_control = 0x32,
	misc_control = 0x34,
	sample_period = 0x36,
	sens_avg = 0x38,
	sleep_count = 0x3a,
	diag_status = 0x3c,
	global_command = 0x3e,
	alarm1_threshold = 0x40,
	alarm2_threshold = 0x42,
	alarm1_sample_size = 0x44,
	alarm2_sample_size = 0x46,
	alarm_control = 0x48,
	aux_dac = 0x4a,
};

#define NOP __nop()

namespace sensors
{

// helpers
#define CSHigh()		cs->write(true)
#define CSLow()			cs->write(false)

#define	SCLKHigh()		sclk->write(true)
#define	SCLKLow()		sclk->write(false)

#define MOSIHigh()		mosi->write(true)
#define MOSILow()		mosi->write(false)

// helpers
static void translate_14bit(signed short *data)
{
	unsigned short *p = (unsigned short *)data;
	if (*p & 0x2000)
		*p |= 0xE000;
	else
		*p &= 0x1FFF;
}

static void DelayCLK()
{
	volatile uint32_t nCount = 100;
	for(; nCount != 0; nCount--);
}
static void DelayBurst()
{
	volatile uint32_t nCount = 12;
	for(; nCount != 0; nCount--);
	//systimer->delayus(1);
}
static void DelayMCU()
{
	volatile uint32_t nCount = 250;
	for(; nCount != 0; nCount--);
}

static void Delay(volatile uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

int16_t ADIS16405::ReadFromADIS16405ViaSpi(unsigned char RegisterAddress)
{
	unsigned	char	ControlValue = 0;
	int16_t 	ReceiveData = 0;
	char		iTemp = 0;			//8位
	int	i = 0, j = 0;
	//先读低8位地址，再读高8位地址

	///Create the 8-bit header
		ControlValue = RegisterAddress ;//every register of ADIS16405 takes up two bytes ，先读低8位地址
		SCLKHigh();
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		CSHigh();
		NOP;
		NOP;
 		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
 		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		CSLow();	 //bring CS low
		DelayCLK();

		//Write out the control word   和地址
		for(i=0; i<16; i++)
		{
			SCLKLow();						//SCL由高变低时写入MISO
			//DelayCLK();
			if(0x80 == (ControlValue & 0x80))
			{
				MOSIHigh();	  //Send one to DIN pin	of ADIS16405
			}
			else
			{
				MOSILow();	  //Send zero to DIN pin of ADIS16405
			}
			DelayCLK();
			SCLKHigh();
			DelayCLK();
			ControlValue <<= 1;	//Rotate data
		}
		DelayCLK();

		//Read data in
		for(i=0; i<16; i++)
		{
			SCLKLow();					
			DelayCLK();
	
			SCLKHigh();			 //由低变高时 采样	  MOSI
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
		
			ReceiveData <<= 1;		//Rotate data
			ReceiveData |= miso->read();
			DelayCLK();

		}
		DelayCLK();
		CSHigh();	//bring CS high again
		translate_14bit(&ReceiveData);
		return ReceiveData;
}


int ADIS16405::adis16405_burst_read(int16_t *out, int count /*=0*/)
{
	if (count == 0 || count >= 12)
		count = 12;
	if (!out)
		return -1;

	CSLow();
	
	systimer->delayus(1);	// tcs

	spi_tx_rx16_burst(0x3E00);
	DelayMCU();
	
	for(int i=0; i<count; i++)
	{
		//DelayCLK();
		out[i] = spi_tx_rx16_burst(0);
	}
	systimer->delayus(1);	
	CSHigh();

	out[0] &= 0x1fff;
	for(int i=1; i<count; i++)
	{
		if (out[i] & 0x2000)
			out[i] |= 0xE000;
		else
			out[i] &= 0x1FFF;
	}

	if (count >= 11)
		if (out[10] & 0x800)
			out[10] |= 0xF000;
		else
			out[10] &= 0x7FF;

	return 0;
}


int ADIS16405::adis16405_read_register(unsigned char registerAddress, unsigned short *out)
{	
	CSLow();
	DelayMCU();
    spi_tx_rx16((registerAddress & 0x7f) << 8);
	DelayMCU();
	*out = spi_tx_rx16(0);
	DelayMCU();
	CSHigh();

	return 0;
}


int ADIS16405::adis16405_write_register(unsigned char registerAddress, unsigned short new_value)
{
	if (registerAddress > aux_dac || (registerAddress & 0x1))
		return -1;

	CSLow();
	DelayMCU();
    spi_tx_rx16((((registerAddress+1) & 0x7f) << 8) | (new_value >> 8) | 0x8000 ) ;
	DelayMCU();
    spi_tx_rx16((((registerAddress) & 0x7f) << 8) | (new_value & 0xff) | 0x8000) ;
	DelayMCU();
	CSHigh();

	return 0;
}

uint16_t ADIS16405::spi_tx_rx16_burst(uint16_t tx)
{
	uint16_t rx = 0;
	int i;

	for(i=0; i<16; i++)
	{
		SCLKLow();

		if (tx & 0x8000)
			MOSIHigh();
		else
			MOSILow();
		tx <<= 1;

		DelayBurst();
		SCLKHigh();
	
		rx <<= 1;
		rx |= miso->read();
		DelayBurst();
	}

	return rx;
}

uint16_t ADIS16405::spi_tx_rx16(uint16_t tx)
{
	uint16_t rx = 0;
	int i;

	for(i=0; i<16; i++)
	{
		SCLKLow();

		if (tx & 0x8000)
			MOSIHigh();
		else
			MOSILow();
		tx <<= 1;

		DelayCLK();
		SCLKHigh();

		rx <<= 1;
		rx |= miso->read();
		DelayCLK();
	}

	return rx;
}

int ADIS16405::init(HAL::IGPIO *sclk, HAL::IGPIO *miso, HAL::IGPIO *mosi, HAL::IGPIO *cs)
{
	this->sclk = sclk;
	this->miso = miso;
	this->mosi = mosi;
	this->cs = cs;

	sclk->set_mode(MODE_OUT_PushPull);
	miso->set_mode(MODE_IN);
	mosi->set_mode(MODE_OUT_PushPull);
	cs->set_mode(MODE_OUT_PushPull);

	unsigned short test2;
	adis16405_read_register(sens_avg, &test2);
	if (test2 != 0x402)
		adis16405_write_register(sens_avg, 0x402);
	adis16405_read_register(sens_avg, &test2);

	m_healthy = test2 == 0x402;

	// read all register
	for(int i=0; i<aux_dac; i+=2)
	{
		short test;
		unsigned short test2;
		systimer->delayms(10);
		test = ReadFromADIS16405ViaSpi(i);
		systimer->delayms(10);
		adis16405_read_register(i, &test2);
		LOGE("%x=%x(%d).%x(%d)\n", i, test, test, test2, test2);
	}

	return m_healthy ? 0 : -1;
}


int ADIS16405::read(devices::accelerometer_data *out)
{
	out->x = data[4] * 0.033f;
	out->y = -data[5] * 0.033f;
	out->z = -data[6] * 0.033f;		// 0.033f : 3.3mg/LSB, g=10
	out->temperature = data[10] * 0.14f + 25;
	
	return 0;
}

int ADIS16405::read(devices::gyro_data *out)
{
	if (adis16405_burst_read(data)<0)
		return -1;
	
	out->x = data[1] * 0.000872665f;
	out->y = -data[2] * 0.000872665f;
	out->z = -data[3] * 0.000872665f;		// 0.000872665f: 0.05 * PI / 180
	out->temperature = data[10] * 0.14f + 25;
	
	return 0;
}


}
