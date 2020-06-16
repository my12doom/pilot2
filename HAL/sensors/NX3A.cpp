#include "NX3A.h"

using namespace HAL;

#define SAMPLEING_TIME 10000 // 10000us, 10ms
#define FAIL_RET(x) if ((x)<0) goto fail

namespace sensors
{

NX3A::NX3A()
{
	_healthy = false;
	temperature = 0;
	pressure = 0;
	new_raw_temperature = 0;
	last_temperature_time = 0;
	last_pressure_time = 0;
}

NX3A::~NX3A()
{
}

int NX3A::init(HAL::II2C *i2c, uint8_t address)
{
	this->i2c = i2c;
	this->address = address;

	uint16_t buffer[0x14];

	FAIL_RET(read_nvm(0x14, (uint16_t*)&nvm14));
	for(int i=0; i<0x14; i++)
		FAIL_RET(read_nvm(i, &buffer[i]));

	
	id = (buffer[0] << 16) | buffer[1];
	version=buffer[0x13]&0x00FF;

	// OSR setting
	nvm14.OSR_Temp = 0;			// OSR=4
	nvm14.OSR_Pressure = 1;		// OSR=32

	// coeff
	t3=((int32_t)((int8_t)((buffer[0x04] & 0x000C) << 4)))<<10 | buffer[0x0E];
	t1=((int32_t)((int8_t)((buffer[0x04] & 0x0003) << 6)))<<10 | buffer[0x0D];
	t2=((int32_t)((int8_t)((buffer[0x04] & 0x0030) << 2)))<<10 | buffer[0x0C];
	p1=((int32_t)((int8_t)((buffer[0x03] & 0x000C) << 4)))<<18 | buffer[0x06] | (buffer[0x10] & 0x00FF) << 16;
	p2=((int32_t)((int8_t)((buffer[0x03] & 0x0003) << 6)))<<18 | buffer[0x05] | (buffer[0x10] & 0xFF00) << 8;
	p3=((int32_t)((int16_t)((buffer[0x03] & 0x3000) << 2))) << 10 | buffer[0x0B] | (buffer[0x13] & 0xFF00) << 8;
	p6=((int32_t)((int16_t)((buffer[0x03] & 0x0C00) << 4))) << 10 | buffer[0x0A] | (buffer[0x12] & 0xFF00) << 8;
	p7=((int32_t)((int16_t)((buffer[0x03] & 0x0300) << 6))) << 10 | (buffer[0x12] & 0x00FF) << 16 | buffer[0x09];
	p4=((int32_t)((int8_t)((buffer[0x03] & 0x0030) << 2))) << 18 | buffer[0x07] | (buffer[0x11] & 0xFF00) << 8;
	p5=((int32_t)((int8_t)((buffer[0x03] & 0x00C0) << 0))) << 18 | buffer[0x08] | (buffer[0x11] & 0x00FF) << 16;
	p8=((int32_t)((int16_t)(buffer[0x03] & 0x8000))) << 1 | buffer[0x0F];
	
	printf("t1,t2,t3,p1,p2,p3,p4,p5,p6,p7,p8=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", t1,t2,t3,p1,p2,p3,p4,p5,p6,p7,p8);

	for(int i=0; i<3; i++)
		FAIL_RET(read_nvm(i+0x1b, &buffer[i]));
	Bshift=(((int32_t)((int16_t)(buffer[0]<<8)))<<8) | buffer[1];
	Tshift=buffer[2];

	// some magic code from official driver
// 	FAIL_RET(wait_chip(1));
// 	FAIL_RET(i2c->start());
// 	FAIL_RET(i2c->txrx(0xa9));
// 	i2c->stop();
// 	while(1)
// 	{
// 		uint16_t tmp = 0x1010;
// 		write_command(0xF1, tmp);
// 		tmp = 0;
// 
// 		read_nv
// 	}

	_healthy = true;

	return true;
fail:
	return false;
}

			double zb, k1, k2;
int NX3A::read(int *data)
{
	int rtn = 1;
	uint8_t tmp[3];

	do
	{
		if (new_raw_temperature == 0 && last_temperature_time == 0 && last_pressure_time == 0)
		{
			if (wait_chip(0) < 0)
			{
				rtn = -1;
				break;
			}
			if (write_command(0xA5, 0x07|(nvm14.OSR_Temp<<6)) < 0)
			{
				rtn = -1;
				break;
			}
			last_temperature_time = systimer->gettime();
		}
		
		if (systimer->gettime() - last_temperature_time >  SAMPLEING_TIME && new_raw_temperature == 0)
		{
			if (wait_chip(0) < 0)
			{
				rtn = -1;
				break;
			}
		
			read_measurement_data(&new_raw_temperature);

			if (write_command(0xA1, *(int16_t*)&nvm14) < 0)
			{
				rtn = -1;
				break;
			}

			last_pressure_time = systimer->gettime();
		}

		if (systimer->gettime() - last_pressure_time >  SAMPLEING_TIME && last_pressure_time > 0)
		{
			if (wait_chip(0) < 0)
			{
				rtn = -1;
				break;
			}
			
			int32_t new_pressure = 0;
			read_measurement_data(&new_pressure);

			// apply calibration
			new_raw_temperature >>= 8;
			if (new_raw_temperature &0x8000)
				new_raw_temperature |= 0xffff0000;
			
			version = 0;
			
			if(version==0)
			{
				zb = t1 / 32768.0 * (new_raw_temperature + t2) + 32768;
				new_temperature = zb * (1 + zb*t3 / 17179869184.0);
			}
			else if(version==1)
			{
				zb=t1/4096.0*(new_raw_temperature+t2);
				new_temperature=zb*(1+zb*t3/536870912.0)+32768;
			}
			
			printf("raw:%d,%d\n", new_raw_temperature, new_pressure);

			zb = new_temperature/16.0 - p8 - 2048;
			k1 = zb*(zb * (p6 / 281474976710656.0) + ( p4 / 137438953472.0)) + 1;
			k2 = p2 + new_pressure + zb*(zb*(p7 / 16777216.0) + (p5 / 4096.0));
			zb = k1 * k2 * (p1 / 4194304.0) + 8192*1024;
			new_pressure = zb * (1 + zb*(p3 / 281474976710656.0));

			new_temperature -= Tshift;
			temperature = ((new_temperature/65536.0)*190-40)*100;
			new_pressure -=Bshift;
			pressure =(((new_pressure)/16777216.0)*100+28.75)*1000/1.125;

			new_raw_temperature = 0;
			last_temperature_time = 0;
			last_pressure_time = 0;
			rtn = 0;
		}
	} while (0);
	
	data[0] = pressure;
	data[1] = temperature;
	return rtn;
}

int NX3A::read(devices::baro_data *out)
{
	int data[2];
	int res = read(data);
	
	out->pressure = data[0];
	out->temperature = data[1] / 100.0f;

	return res;
}

bool NX3A::healthy()
{
	return _healthy;
}

// stupid IIC commands
int NX3A::read_status()
{
	int status;

	FAIL_RET(i2c->start());
	FAIL_RET(i2c->tx(address | 0x1));
	FAIL_RET(i2c->wait_ack());
	status=i2c->rx();
	FAIL_RET(i2c->send_nak());
fail:
	i2c->stop();

	return status;
}
int NX3A::read_nvm(uint8_t nvm_address, uint16_t *out)
{
	uint8_t status;

	FAIL_RET(i2c->start());
	FAIL_RET(i2c->tx(address));
	FAIL_RET(i2c->wait_ack());
	i2c->rx();
	FAIL_RET(i2c->wait_ack());
	FAIL_RET(i2c->stop());

	systimer->delayus(85);

	FAIL_RET(i2c->start());
	FAIL_RET(i2c->tx(address | 0x1));
	FAIL_RET(i2c->wait_ack());
	status =i2c->rx();
	FAIL_RET(i2c->send_ack());
	out[0] = i2c->rx() << 8;
	FAIL_RET(i2c->send_ack());
	out[0] |= i2c->rx();
	FAIL_RET(i2c->send_nak());
fail:
	i2c->stop();

	return 0;
}

int NX3A::write_command(uint8_t cmd, uint16_t cmd_data)
{
	FAIL_RET(i2c->start());
	FAIL_RET(i2c->tx(address));
	FAIL_RET(i2c->wait_ack());
	i2c->tx(cmd);
	FAIL_RET(i2c->wait_ack());
	i2c->tx(cmd_data >> 8);
	FAIL_RET(i2c->wait_ack());
	i2c->tx(cmd & 0xff);
	FAIL_RET(i2c->wait_ack());
fail:
	i2c->stop();

	return 0;
}
int NX3A::read_measurement_data(int32_t *out)
{
	uint8_t status;
	FAIL_RET(i2c->start());

	FAIL_RET(i2c->tx(address | 0x1));
	FAIL_RET(i2c->wait_ack());

	status =i2c->rx();
	FAIL_RET(i2c->send_ack());

	*out |= i2c->rx();
	*out <<= 8; 
	FAIL_RET(i2c->send_ack());

	*out |= i2c->rx();
	*out <<= 8; 
	FAIL_RET(i2c->send_ack());

	*out |= i2c->rx();
	FAIL_RET(i2c->send_nak());
fail:
	i2c->stop();

	if (*out & 0x800000)
		*out |= 0xff000000;

	return 0;
}

int NX3A::wait_chip(int timeout)
{
	do
	{
		int status = read_status();
		if (status < 0)
			return status;

		if (!(status & 0x20))
			return 0;
	}
	while(timeout--);

	return -1;
}

}