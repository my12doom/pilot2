#include "OLED_I2C.h"
#include <HAL/Interface/ISysTimer.h>
#include "codetab.h"
#include <string.h>

namespace devices
{

#define FAIL_RETURN(x) if((x)<0) return-1

int OLED96::init(HAL::II2C *i2c, uint8_t address)
{
	this->i2c = i2c;
	this->address = address;

	systimer->delayms(500);

	FAIL_RETURN(WriteCmd(0xae));//--turn off oled panel
	FAIL_RETURN(WriteCmd(0x00));//---set 0 column address
	FAIL_RETURN(WriteCmd(0x10));//---set 1 column address
	FAIL_RETURN(WriteCmd(0x40));//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	FAIL_RETURN(WriteCmd(0x81));//--set contrast control register
	FAIL_RETURN(WriteCmd(0xff)); // Set SEG Output Current Brightness
	FAIL_RETURN(WriteCmd(0xa1));//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	FAIL_RETURN(WriteCmd(0xc8));//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	FAIL_RETURN(WriteCmd(0xa6));//--set normal display
	FAIL_RETURN(WriteCmd(0xa8));//--set multiplex ratio(1 to 64)
	FAIL_RETURN(WriteCmd(0x3f));//--1/64 duty
	FAIL_RETURN(WriteCmd(0xd3));//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	FAIL_RETURN(WriteCmd(0x00));//-not offset
	FAIL_RETURN(WriteCmd(0xd5));//--set display clock divide ratio/oscillator frequency
	FAIL_RETURN(WriteCmd(0x80));//--set divide ratio, Set Clock as 100 Frames/Sec
	FAIL_RETURN(WriteCmd(0xd9));//--set pre-charge period
	FAIL_RETURN(WriteCmd(0xf1));//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	FAIL_RETURN(WriteCmd(0xda));//--set com pins hardware configuration
	FAIL_RETURN(WriteCmd(0x12));
	FAIL_RETURN(WriteCmd(0xdb));//--set vcomh
	FAIL_RETURN(WriteCmd(0x40));//Set VCOM Deselect Level
	FAIL_RETURN(WriteCmd(0x20));//-Set Page Addressing Mode (0x00/0x01/0x02)
	FAIL_RETURN(WriteCmd(0x02));//
	FAIL_RETURN(WriteCmd(0x8d));//--set Charge Pump enable/disable
	FAIL_RETURN(WriteCmd(0x14));//--set(0x10) disable
	FAIL_RETURN(WriteCmd(0xa4));// Disable Entire Display On (0xa4/0xa5)
	FAIL_RETURN(WriteCmd(0xa6));// Disable Inverse Display On (0xa6/a7) 
	FAIL_RETURN(WriteCmd(0xaf));//--turn on oled panel

	fill(0); //初始清屏
	move(0,0);

	return 0;
}

int OLED96::move(int x, int y)
{
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);

	return 0;
}

int OLED96::fill(uint8_t color)
{
	uint8_t data[128];
	memset(data, color, 128);
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address

		i2c->write_regs(address, 0x40, data, 128);
	}


	return 0;
}

int OLED96::clear()
{
	fill(0);

	return 0;
}

int OLED96::on()
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED唤醒
	return 0;
}

int OLED96::off()
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
	return 0;
}

int OLED96::show_str(int x, int y, const char *text)
{
	unsigned char c = 0,i = 0,j = 0;

	while(text[j] != '\0')
	{
		c = text[j] - 32;
		if(x > 126)
		{
			x = 0;
			y++;
		}
		move(x,y);
		for(i=0;i<6;i++)
			write_pixel(F6x8[c][i]);
		x += 6;
		j++;
	}

	return 0;
}

int OLED96::draw_bmp(int width, int height, const void *data, int x, int y)
{
	unsigned int j=0;
	uint8_t *BMP = (uint8_t*)data;

	for(int py=0; py<height; py++)
	{
		move(x,y+py);
		for(int px=0; px<width; px++)
		{
			write_pixel(BMP[j++]);
		}
	}

	return 0;
}

int OLED96::WriteCmd(uint8_t cmd)
{
	return i2c->write_reg(address, 0, cmd);
}
int OLED96::write_pixel(uint8_t data)
{
	return i2c->write_reg(address, 0x40, data);
}


}