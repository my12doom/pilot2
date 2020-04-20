#pragma once

#include <HAL/Interface/II2C.h>

namespace devices
{
class OLED96
{
public:
	OLED96(){}
	~OLED96(){}

	int init(HAL::II2C *i2c, uint8_t address);

	int move(int x, int y);
	int fill(uint8_t color);
	int clear();
	int on();
	int off();
	int show_str(int x, int y, const char *text);
	int draw_bmp(int width, int height, const void *data, int x, int y);
	int write_pixel(uint8_t data);

protected:
	int WriteCmd(uint8_t cmd);
	uint8_t address;
	HAL::II2C *i2c;
};
}
