#include <string.h>

#include <stdint.h>
#include <HAL/Interface/II2C.h>
#include <HAL/STM32F4/F4GPIO.h>

using namespace STM32F4;
using namespace HAL;
	
uint8_t edit[256];
uint8_t eepromdat[256] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x10, 0xAC, 0xA3, 0xA0, 0x4C, 0x31, 0x4A, 0x39, 0x16, 0x18, 0x01, 0x04, 0xA2, 0x35, 0x1E, 0x78, 0x06, 0x7E, 0x75, 0xA7, 0x55, 0x52, 0x9C, 0x27, 0x0F, 0x50, 0x54, 0xA5, 0x4B, 0x00, 0x71, 0x4F, 0x81, 0x80, 0xA9, 0xC0, 0xA9, 0x40, 0xD1, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C, 0x45, 0x00, 0x0F, 0x28, 0x21, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x34, 0x43, 0x57, 0x58, 0x37, 0x34, 0x35, 0x51, 0x39, 0x4A, 0x31, 0x4C, 0x0A, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x44, 0x45, 0x4C, 0x4C, 0x20, 0x55, 0x32, 0x34, 0x31, 0x34, 0x48, 0x0A, 0x20, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x38, 0x4C, 0x1E, 0x53, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };


int r = 0;
void read_edid()
{
	F4GPIO SCL(GPIOB, GPIO_Pin_6);
	F4GPIO SDA(GPIOB, GPIO_Pin_7);
	I2C_SW i2c(&SCL, &SDA);
	
	i2c.set_speed(200);
	for(int i=0; i<256; i++)
	{
		//r = i2c.write_reg(0xA0, i, eepromdat[i]);
		//systimer->delayms(10);
		r = i2c.read_reg(0xA0, i, &edit[i]);
	}
	
	r = memcmp(eepromdat, edit, sizeof(edit));
	
	return;
}

unsigned char adv7611_script[] = {
	0x98, 0xF4, 0x80,
	0x98, 0xF5, 0x7C,
	0x98, 0xF8, 0x4C,
	0x98, 0xF9, 0x64,
	0x98, 0xFA, 0x6C,
	0x98, 0xFB, 0x68,
	0x98, 0xFD, 0x44,
	0x98, 0x01, 0x06,
	0x98, 0x02, 0xF7,
	0x98, 0x03, 0x40,
	0x98, 0x04, 0x42,
	0x98, 0x05, 0x28,
	0x98, 0x06, 0xA6,
	0x98, 0x0B, 0x44,
	0x98, 0x0C, 0x42,
	0x98, 0x15, 0x80,
	0x98, 0x19, 0x8A,
	0x98, 0x14, 0x7F,
	0x98, 0x33, 0x40,
	0x44, 0xBA, 0x01,
	0x64, 0x40, 0x81,
	0x68, 0x9B, 0x03,
	0x68, 0xC1, 0x01,
	0x68, 0xC2, 0x01,
	0x68, 0xC3, 0x01,
	0x68, 0xC4, 0x01,
	0x68, 0xC5, 0x01,
	0x68, 0xC6, 0x01,
	0x68, 0xC7, 0x01,
	0x68, 0xC8, 0x01,
	0x68, 0xC9, 0x01,
	0x68, 0xCA, 0x01,
	0x68, 0xCB, 0x01,
	0x68, 0xCC, 0x01,
	0x68, 0x00, 0x00,
	0x68, 0x83, 0xFE,
	0x68, 0x6F, 0x08,
	0x68, 0x85, 0x1F,
	0x68, 0x87, 0x70,
	0x68, 0x8D, 0x04,
	0x68, 0x8E, 0x1E,
	0x68, 0x1A, 0x8A,
	0x68, 0x57, 0xDA,
	0x68, 0x58, 0x01,
	0x68, 0x75, 0x10,
	0x34, 0x00, 0x80,
	0x34, 0x02, 0x80,
	0x34, 0x08, 0x10,
	0x34, 0x0A, 0x06,
	0x34, 0x0C, 0x17,
	0x34, 0x0E, 0x0A,
	0x34, 0x12, 0x01,
	0xFF };


int main()
{
	F4GPIO SCL(GPIOB, GPIO_Pin_2);
	F4GPIO SDA(GPIOB, GPIO_Pin_1);
	I2C_SW i2c_adv7611(&SCL, &SDA);
	i2c_adv7611.set_speed(20);
	
	/*
	uint8_t cec_i2c = 0x80;				// 0xf4
	uint8_t info_i2c = 0x7c;			// 0xf5
	
	uint8_t repteater_i2c = 0x64;
	uint8_t edid_ram_i2c = 0x6c;
	uint8_t hdmi_i2c = 0x68;
	uint8_t cp_i2c = 0x68;
	
	i2c.write_reg(0x98, 0xfa, edid_ram_i2c);
	i2c.write_reg(0x98, 0xf9, repteater_i2c);
	
	
	i2c.write_reg(repteater_i2c, 0x74, 1);		// disable EDID readout on DDC.
	
	for(int i=0; i<256; i++)
		r = i2c.write_reg(0x9a, i, eepromdat[i]);		
	
	i2c.write_reg(repteater_i2c, 0x74, 1);		// enable EDID readout on DDC.
	*/
	
	// reset ADV7611
	r = i2c_adv7611.write_reg(0x98, 0xFF, 0x80);		
	systimer->delayms(1);
	
	// program ADV7611 script
	for(int i=0; i<sizeof(adv7611_script); i+=3)
	{
		if (adv7611_script[i] == 0xff)
			break;
		
		r = i2c_adv7611.write_reg(adv7611_script[i], adv7611_script[i+1], adv7611_script[i+2]);
	}
	
	// program EDID
	r = i2c_adv7611.write_reg(0x68, 0x6C, 0xA3); // enable manual HPA
	r = i2c_adv7611.write_reg(0x98, 0x20, 0x70); // HPD low
	r = i2c_adv7611.write_reg(0x64, 0x74, 0x00); // disable internal EDID
	
	for(int i=0; i<256; i++)
		r = i2c_adv7611.write_reg(0x6C, i, eepromdat[i]);
	
	systimer->delayms(100);
	
	r = i2c_adv7611.write_reg(0x64, 0x74, 0x01); // enable internal EDID
	r = i2c_adv7611.write_reg(0x98, 0x20, 0xF0); // HPD high
	r = i2c_adv7611.write_reg(0x68, 0x6C, 0xA3); // disable manual HPA
	
	
	while(1)
	{
	}

}