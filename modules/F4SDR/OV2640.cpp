#include <stdlib.h>
#include "OV2640.h"
#include "dcmi_ov2640.h"
#include <stm32f4xx_dma.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_dcmi.h>

using namespace HAL;

#define FAIL_RETURN(x) if((x)<0) return -1

const unsigned char OV2640_JPEG_INIT[][2]=
{
	0xff, 0x00,
	0x2c, 0xff,
	0x2e, 0xdf,
	0xff, 0x01,
	0x3c, 0x32,
	0x11, 0x00,
	0x09, 0x02,
	0x04, 0x28,
	0x13, 0xe5,
	0x14, 0x48,
	0x2c, 0x0c,
	0x33, 0x78,
	0x3a, 0x33,
	0x3b, 0xfB,
	0x3e, 0x00,
	0x43, 0x11,
	0x16, 0x10,
	0x39, 0x92,
	0x35, 0xda,
	0x22, 0x1a,
	0x37, 0xc3,
	0x23, 0x00,
	0x34, 0xc0,
	0x36, 0x1a,
	0x06, 0x88,
	0x07, 0xc0,
	0x0d, 0x87,
	0x0e, 0x41,
	0x4c, 0x00,
	0x48, 0x00,
	0x5B, 0x00,
	0x42, 0x03,
	0x4a, 0x81,
	0x21, 0x99,
	0x24, 0x40,
	0x25, 0x38,
	0x26, 0x82,
	0x5c, 0x00,
	0x63, 0x00,
	0x61, 0x70,
	0x62, 0x80,
	0x7c, 0x05,
	0x20, 0x80,
	0x28, 0x30,
	0x6c, 0x00,
	0x6d, 0x80,
	0x6e, 0x00,
	0x70, 0x02,
	0x71, 0x94,
	0x73, 0xc1,
	0x12, 0x40,//0x40
	0x17, 0x11,
	0x18, 0x43,
	0x19, 0x00,
	0x1a, 0x4b,
	0x32, 0x09,
	0x37, 0xc0,
	0x4f, 0x60,
	0x50, 0xa8,
	0x6d, 0x00,
	0x3d, 0x38,
	0x46, 0x3f,
	0x4f, 0x60,
	0x0c, 0x3c,
	0xff, 0x00,
	0xe5, 0x7f,
	0xf9, 0xc0,
	0x41, 0x24,
	0xe0, 0x14,
	0x76, 0xff,
	0x33, 0xa0,
	0x42, 0x20,
	0x43, 0x18,
	0x4c, 0x00,
	0x87, 0xd5,
	0x88, 0x3f,
	0xd7, 0x03,
	0xd9, 0x10,
	0xd3, 0x82,
	0xc8, 0x08,
	0xc9, 0x80,
	0x7c, 0x00,
	0x7d, 0x00,
	0x7c, 0x03,
	0x7d, 0x48,
	0x7d, 0x48,
	0x7c, 0x08,
	0x7d, 0x20,
	0x7d, 0x10,
	0x7d, 0x0e,
	0x90, 0x00,
	0x91, 0x0e,
	0x91, 0x1a,
	0x91, 0x31,
	0x91, 0x5a,
	0x91, 0x69,
	0x91, 0x75,
	0x91, 0x7e,
	0x91, 0x88,
	0x91, 0x8f,
	0x91, 0x96,
	0x91, 0xa3,
	0x91, 0xaf,
	0x91, 0xc4,
	0x91, 0xd7,
	0x91, 0xe8,
	0x91, 0x20,
	0x92, 0x00,
	0x93, 0x06,
	0x93, 0xe3,
	0x93, 0x05,
	0x93, 0x05,
	0x93, 0x00,
	0x93, 0x04,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x96, 0x00,
	0x97, 0x08,
	0x97, 0x19,
	0x97, 0x02,
	0x97, 0x0c,
	0x97, 0x24,
	0x97, 0x30,
	0x97, 0x28,
	0x97, 0x26,
	0x97, 0x02,
	0x97, 0x98,
	0x97, 0x80,
	0x97, 0x00,
	0x97, 0x00,
	0xc3, 0xed,
	0xa4, 0x00,
	0xa8, 0x00,
	0xc5, 0x11,
	0xc6, 0x51,
	0xbf, 0x80,
	0xc7, 0x10,
	0xb6, 0x66,
	0xb8, 0xA5,
	0xb7, 0x64,
	0xb9, 0x7C,
	0xb3, 0xaf,
	0xb4, 0x97,
	0xb5, 0xFF,
	0xb0, 0xC5,
	0xb1, 0x94,
	0xb2, 0x0f,
	0xc4, 0x5c,
	0xc0, 0x64,
	0xc1, 0x4B,
	0x8c, 0x00,
	0x86, 0x3D,
	0x50, 0x00,
	0x51, 0xC8,
	0x52, 0x96,
	0x53, 0x00,
	0x54, 0x00,
	0x55, 0x00,
	0x5a, 0xC8,
	0x5b, 0x96,
	0x5c, 0x00,
	0xd3, 0x7f,
	0xc3, 0xed,
	0x7f, 0x00,
	0xda, 0x00,
	0xe5, 0x1f,
	0xe1, 0x67,
	0xe0, 0x00,
	0xdd, 0x7f,
	0x05, 0x00,

	0x12, 0x40,//0x40
	0xd3, 0x7f,
	0xc0, 0x16,
	0xC1, 0x12,
	0x8c, 0x00,
	0x86, 0x3d,
	0x50, 0x00,
	0x51, 0x2C,
	0x52, 0x24,
	0x53, 0x00,
	0x54, 0x00,
	0x55, 0x00,
	0x5A, 0x2c,
	0x5b, 0x24,
	0x5c, 0x00,
};
const u8 ov2640_svga_init_reg_tbl[][2]= 
{    
	0xff, 0x00,
	0x2c, 0xff,
	0x2e, 0xdf,
	0xff, 0x01,
	0x3c, 0x32,
	//
	// 0x11, 0x06, // ???fps
	0x11, 0x01,		// 14.28fps
	0x09, 0x02,
	0x04, 0xD8,//ˮƽ����,��ֱ��ת
	0x13, 0xe5,
	0x14, 0x48,
	0x2c, 0x0c,
	0x33, 0x78,
	0x3a, 0x33,
	0x3b, 0xfB,
	//
	0x3e, 0x00,
	0x43, 0x11,
	0x16, 0x10,
	//
	0x39, 0x92,
	//
	0x35, 0xda,
	0x22, 0x1a,
	0x37, 0xc3,
	0x23, 0x00,
	0x34, 0xc0,
	0x36, 0x1a,
	0x06, 0x88,
	0x07, 0xc0,
	0x0d, 0x87,
	0x0e, 0x41,
	0x4c, 0x00,
	0x48, 0x00,
	0x5B, 0x00,
	0x42, 0x03,
	//
	0x4a, 0x81,
	0x21, 0x99,
	//
	0x24, 0x40,
	0x25, 0x38,
	0x26, 0x82,
	0x5c, 0x00,
	0x63, 0x00,
	0x46, 0x22,
	0x0c, 0x3c,
	//
	0x61, 0x70,
	0x62, 0x80,
	0x7c, 0x05,
	//
	0x20, 0x80,
	0x28, 0x30,
	0x6c, 0x00,
	0x6d, 0x80,
	0x6e, 0x00,
	0x70, 0x02,
	0x71, 0x94,
	0x73, 0xc1,
	
	0x3d, 0x34, 
	0x5a, 0x57,
	//���ݷֱ��ʲ�ͬ������
	0x12, 0x40,//SVGA 800*600
	0x17, 0x11,
	0x18, 0x43,
	0x19, 0x00,
	0x1a, 0x4b,
	0x32, 0x09,
	0x37, 0xc0,
	//
	0x4f, 0xca,
	0x50, 0xa8,
	0x5a, 0x23,
	0x6d, 0x00,
	0x3d, 0x38,
	//
	0xff, 0x00,
	0xe5, 0x7f,
	0xf9, 0xc0,
	0x41, 0x24,
	0xe0, 0x14,
	0x76, 0xff,
	0x33, 0xa0,
	0x42, 0x20,
	0x43, 0x18,
	0x4c, 0x00,
	0x87, 0xd5,
	0x88, 0x3f,
	0xd7, 0x03,
	0xd9, 0x10,
	0xd3, 0x82,
	//
	0xc8, 0x08,
	0xc9, 0x80,
	//
	0x7c, 0x00,
	0x7d, 0x00,
	0x7c, 0x03,
	0x7d, 0x48,
	0x7d, 0x48,
	0x7c, 0x08,
	0x7d, 0x20,
	0x7d, 0x10,
	0x7d, 0x0e,
	//
	0x90, 0x00,
	0x91, 0x0e,
	0x91, 0x1a,
	0x91, 0x31,
	0x91, 0x5a,
	0x91, 0x69,
	0x91, 0x75,
	0x91, 0x7e,
	0x91, 0x88,
	0x91, 0x8f,
	0x91, 0x96,
	0x91, 0xa3,
	0x91, 0xaf,
	0x91, 0xc4,
	0x91, 0xd7,
	0x91, 0xe8,
	0x91, 0x20,
	//
	0x92, 0x00,
	0x93, 0x06,
	0x93, 0xe3,
	0x93, 0x05,
	0x93, 0x05,
	0x93, 0x00,
	0x93, 0x04,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	//
	0x96, 0x00,
	0x97, 0x08,
	0x97, 0x19,
	0x97, 0x02,
	0x97, 0x0c,
	0x97, 0x24,
	0x97, 0x30,
	0x97, 0x28,
	0x97, 0x26,
	0x97, 0x02,
	0x97, 0x98,
	0x97, 0x80,
	0x97, 0x00,
	0x97, 0x00,
	//
	0xc3, 0xed,
	0xa4, 0x00,
	0xa8, 0x00,
	0xc5, 0x11,
	0xc6, 0x51,
	0xbf, 0x80,
	0xc7, 0x10,
	0xb6, 0x66,
	0xb8, 0xA5,
	0xb7, 0x64,
	0xb9, 0x7C,
	0xb3, 0xaf,
	0xb4, 0x97,
	0xb5, 0xFF,
	0xb0, 0xC5,
	0xb1, 0x94,
	0xb2, 0x0f,
	0xc4, 0x5c,
	//���ݷֱ��ʲ�ͬ������
	0xc0, 0x64,
	0xc1, 0x4B,
	0x8c, 0x00,
	0x86, 0x3D,
	0x50, 0x00,
	0x51, 0xC8,
	0x52, 0x96,
	0x53, 0x00,
	0x54, 0x00,
	0x55, 0x00,
	0x5a, 0xC8,
	0x5b, 0x96,
	0x5c, 0x00,
	
	0xd3, 0x02,//auto����ҪС��
	//
	0xc3, 0xed,
	0x7f, 0x00,
	
	0xda, 0x09,
	
	0xe5, 0x1f,
	0xe1, 0x67,
	0xe0, 0x00,
	0xdd, 0x7f,
	0x05, 0x00,
};
const unsigned char OV2640_YUV422[][2]= 
{
	0xFF, 0x00,
	0x05, 0x00,
	0xDA, 0x10,
	0xD7, 0x03,
	0xDF, 0x00,
	0x33, 0x80,
	0x3C, 0x40,
	0xe1, 0x77,
	0x00, 0x00,
};

const unsigned char OV2640_JPEG[][2]=
{
	0xe0, 0x14,
	0xe1, 0x77,
	0xe5, 0x1f,
	0xd7, 0x03,
	0xda, 0x10,
	0xe0, 0x00,
	0xFF, 0x01,
	0x04, 0x08,
};
const unsigned char set_win_off_before[][2] = 
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02Ϊ����
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	0xff,      0x00,
	0xe0,      0x04,
	0xc0,      0xc8,
	0xc1,      0x96,
	0x86,      0x3d,
	0x50,      0x92,
//	0x51,      0x90,
//	0x52,      0x2c,
//	0x53,      0x00,
//	0x54,      0x00,
//	0x55,      0x88,
//	0x57,      0x00,
	0x5a,      0x50,
	0x5b,      0x3c,
	0x5c,      0x00,
	0xd3,      0x7F,
};
/* JPG 320x240 */
const unsigned char OV2640_320x240_JPEG[][2]=
{
	{0xff, 0x01},
	{0x11, 0x01},
	{0x12, 0x00}, // Bit[6:4]: Resolution selection//0x02Ϊ����
	{0x17, 0x11}, // HREFST[10:3]
	{0x18, 0x75}, // HREFEND[10:3]
	{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	{0x19, 0x01}, // VSTRT[9:2]
	{0x1a, 0x97}, // VEND[9:2]
	{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	{0x37, 0x40},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0x5a, 0x57},
	{0x6d, 0x80},
	{0x3d, 0x34},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},

	////////////////
	/*
	//176*144
	0xff,      0x00,
	0xc0,      0xC8,
	0xc1,      0x96,
	0x8c,      0x00,
	0x86,      0x3D,
	0x50,      0x9B,
	0x51,      0x90,
	0x52,      0x2C,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x5a,      0x2C,
	0x5b,      0x24,
	0x5c,      0x00,
	0xd3,      0x7F,
	////////////
	*/

	////////////////
	//320*240
	0xff,      0x00,
	0xe0,      0x04,
	0xc0,      0xc8,
	0xc1,      0x96,
	0x86,      0x3d,
	0x50,      0x92,
	0x51,      0x90,
	0x52,      0x2c,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x57,      0x00,
	0x5a,      0x50,
	0x5b,      0x3c,
	0x5c,      0x00,
	0xd3,      0x7F,
	0xe0,      0x00,
	///////////////////

	/*
	0xff,      0x00,
	0xe0,      0x04,
	0xc0,      0xc8,
	0xc1,      0x96,
	0x86,      0x35,
	0x50,      0x92,
	0x51,      0x90,
	0x52,      0x2c,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x57,      0x00,
	0x5a,      0x58,
	0x5b,      0x48,
	0x5c,      0x00,
	0xd3,      0x08,
	0xe0,      0x00
	*/
	/*
	//640*480	  
	0xff,      0x00,
	0xe0,      0x04,
	0xc0,      0xc8,
	0xc1,      0x96,
	0x86,      0x3d,
	0x50,      0x89,
	0x51,      0x90,
	0x52,      0x2c,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x57,      0x00,
	0x5a,      0xa0,
	0x5b,      0x78,
	0x5c,      0x00,
	0xd3,      0x04,
	0xe0,      0x00
	*/
	/////////////////////
	/*
	//800*600
	0xff,      0x00,
	0xe0,      0x04,
	0xc0,      0xc8,
	0xc1,      0x96,
	0x86,      0x35,
	0x50,      0x89,
	0x51,      0x90,
	0x52,      0x2c,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x57,      0x00,
	0x5a,      0xc8,
	0x5b,      0x96,
	0x5c,      0x00,
	0xd3,      0x02,
	0xe0,      0x00
	*/
	/*
	//1280*1024

	0xff,      0x00,
	0xe0,      0x04,
	0xc0,      0xc8,
	0xc1,      0x96,
	0x86,      0x3d,
	0x50,      0x00,
	0x51,      0x90,
	0x52,      0x2c,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x57,      0x00,
	0x5a,      0x40,
	0x5b,      0xf0,
	0x5c,      0x01,
	0xd3,      0x02,
	0xe0,      0x00
	*/
	/*
	/////////////////////
	//1600*1200

	0xff,      0x00,
	0xe0,      0x04,
	0xc0,      0xc8,
	0xc1,      0x96,
	0x86,      0x3d,
	0x50,      0x00,
	0x51,      0x90,
	0x52,      0x2c,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x57,      0x00,
	0x5a,      0x90,
	0x5b,      0x2C,
	0x5c,      0x05,//bit2->1;bit[1:0]->1
	0xd3,      0x02,
	0xe0,      0x00
	/////////////////////
	*/
	/*
	//1024*768
	0xff,      0x00,
	0xc0,      0xC8,
	0xc1,      0x96,
	0x8c,      0x00,
	0x86,      0x3D,
	0x50,      0x00,
	0x51,      0x90,
	0x52,      0x2C,
	0x53,      0x00,
	0x54,      0x00,
	0x55,      0x88,
	0x5a,      0x00,
	0x5b,      0xC0,
	0x5c,      0x01,
	0xd3,      0x02
	*/
};
const u8 OV2640_640x480_JPEG[][2]=
{
    0xff,  0x01,
    0x11,  0x06,
    0x12,  0x00,
    0x17,  0x11,
    0x18,  0x75,
    0x32,  0x36,
    0x19,  0x01,
    0x1a,  0x97,
    0x03,  0x0f,
    0x37,  0x40,
    0x4f,  0xbb,
    0x50,  0x9c,
    0x5a,  0x57,
    0x6d,  0x80,
    0x3d,  0x34,
    0x39,  0x02,
    0x35,  0x88,
    0x22,  0x0a,
    0x37,  0x40,
    0x34,  0xa0,
    0x06,  0x02,
    0x0d,  0xb7,
    0x0e,  0x01,

    0xff,  0x00,
    0xe0,  0x04,
    0xc0,  0xc8,
    0xc1,  0x96,
    0x86,  0x3d,
    0x50,  0x89,
    0x51,  0x90,
    0x52,  0x2c,
    0x53,  0x00,
    0x54,  0x00,
    0x55,  0x88,
    0x57,  0x00,
    0x5a,  0xa0,
    0x5b,  0x78,
    0x5c,  0x00,
    0xd3,  0x7f,
    0xe0,  0x00,
};
/* JPG 352x288 */
const unsigned char OV2640_352x288_JPEG[][2]=
{
0xff, 0x01,
  0x12, 0x40,
  0x17, 0x11,
  0x18, 0x43,
  0x19, 0x00,
  0x1a, 0x4b,
  0x32, 0x09,
  0x4f, 0xca,
  0x50, 0xa8,
  0x5a, 0x23,
  0x6d, 0x00,
  0x39, 0x12,
  0x35, 0xda,
  0x22, 0x1a,
  0x37, 0xc3,
  0x23, 0x00,
  0x34, 0xc0,
  0x36, 0x1a,
  0x06, 0x88,
  0x07, 0xc0,
  0x0d, 0x87,
  0x0e, 0x41,
  0x4c, 0x00,
  
  0xff, 0x00,
  0xe0, 0x04,
  0xc0, 0x64,
  0xc1, 0x4b,
  0x86, 0x35,
  0x50, 0x89,
  0x51, 0xc8,
  0x52, 0x96,
  0x53, 0x00,
  0x54, 0x00,
  0x55, 0x00,
  0x57, 0x00,
  0x5a, 0x58,
  0x5b, 0x48,
  0x5c, 0x00,
  0xe0, 0x00,
};
const u8 ov2640_sxga_init_reg_tbl[][2]= 
{   
	0xff, 0x00,
	0x2c, 0xff,
	0x2e, 0xdf,
	0xff, 0x01,
	0x3c, 0x32,
	//
	0x11, 0x00,	// 15fps
	0x09, 0x02,
	0x04, 0xD8,//ˮƽ����,��ֱ��ת
	0x13, 0xe5,
	0x14, 0x48,
	0x2c, 0x0c,
	0x33, 0x78,
	0x3a, 0x33,
	0x3b, 0xfB,
	//
	0x3e, 0x00,
	0x43, 0x11,
	0x16, 0x10,
	//
	0x39, 0x92,
	//
	0x35, 0xda,
	0x22, 0x1a,
	0x37, 0xc3,
	0x23, 0x00,
	0x34, 0xc0,
	0x36, 0x1a,
	0x06, 0x88,
	0x07, 0xc0,
	0x0d, 0x87,
	0x0e, 0x41,
	0x4c, 0x00,
	
	0x48, 0x00,
	0x5B, 0x00,
	0x42, 0x03,
	//
	0x4a, 0x81,
	0x21, 0x99,
	//
	0x24, 0x40,
	0x25, 0x38,
	0x26, 0x82,
	0x5c, 0x00,
	0x63, 0x00,
	0x46, 0x00,
	0x0c, 0x3c,
	//
	0x61, 0x70,
	0x62, 0x80,
	0x7c, 0x05,
	//
	0x20, 0x80,
	0x28, 0x30,
	0x6c, 0x00,
	0x6d, 0x80,
	0x6e, 0x00,
	0x70, 0x02,
	0x71, 0x94,
	0x73, 0xc1, 
	0x3d, 0x34, 
	0x5a, 0x57,
	//
	0x12, 0x00,//UXGA 1600*1200
	
	0x17, 0x11,
	0x18, 0x75,
	0x19, 0x01,
	0x1a, 0x97,
	0x32, 0x36,
	0x03, 0x0f, 
	0x37, 0x40,
	// 
	0x4f, 0xca,
	0x50, 0xa8,
	0x5a, 0x23,
	0x6d, 0x00,
	0x6d, 0x38,
	//
	0xff, 0x00,
	0xe5, 0x7f,
	0xf9, 0xc0,
	0x41, 0x24,
	0xe0, 0x14,
	0x76, 0xff,
	0x33, 0xa0,
	0x42, 0x20,
	0x43, 0x18,
	0x4c, 0x00,
	0x87, 0xd5,
	0x88, 0x3f,
	0xd7, 0x03,
	0xd9, 0x10,
	0xd3, 0x82,
	//
	0xc8, 0x08,
	0xc9, 0x80,
	//
	0x7c, 0x00,
	0x7d, 0x00,
	0x7c, 0x03,
	0x7d, 0x48,
	0x7d, 0x48,
	0x7c, 0x08,
	0x7d, 0x20,
	0x7d, 0x10,
	0x7d, 0x0e,
	//
	0x90, 0x00,
	0x91, 0x0e,
	0x91, 0x1a,
	0x91, 0x31,
	0x91, 0x5a,
	0x91, 0x69,
	0x91, 0x75,
	0x91, 0x7e,
	0x91, 0x88,
	0x91, 0x8f,
	0x91, 0x96,
	0x91, 0xa3,
	0x91, 0xaf,
	0x91, 0xc4,
	0x91, 0xd7,
	0x91, 0xe8,
	0x91, 0x20,
	//
	0x92, 0x00,
	0x93, 0x06,
	0x93, 0xe3,
	0x93, 0x05,
	0x93, 0x05,
	0x93, 0x00,
	0x93, 0x04,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	0x93, 0x00,
	//
	0x96, 0x00,
	0x97, 0x08,
	0x97, 0x19,
	0x97, 0x02,
	0x97, 0x0c,
	0x97, 0x24,
	0x97, 0x30,
	0x97, 0x28,
	0x97, 0x26,
	0x97, 0x02,
	0x97, 0x98,
	0x97, 0x80,
	0x97, 0x00,
	0x97, 0x00,
	//
	0xc3, 0xef,
	
	0xa4, 0x00,
	0xa8, 0x00,
	0xc5, 0x11,
	0xc6, 0x51,
	0xbf, 0x80,
	0xc7, 0x10,
	0xb6, 0x66,
	0xb8, 0xA5,
	0xb7, 0x64,
	0xb9, 0x7C,
	0xb3, 0xaf,
	0xb4, 0x97,
	0xb5, 0xFF,
	0xb0, 0xC5,
	0xb1, 0x94,
	0xb2, 0x0f,
	0xc4, 0x5c,
	//
	0xc0, 0xc8,
	0xc1, 0x96,
	0x8c, 0x00,
	0x86, 0x3d,
	0x50, 0x00,
	0x51, 0x90,
	0x52, 0x2c,
	0x53, 0x00,
	0x54, 0x00,
	0x55, 0x88,
	
	0x5a, 0x90,
	0x5b, 0x2C,
	0x5c, 0x05,
	
	0xd3, 0x02,//auto����ҪС��
	//
	0xc3, 0xed,
	0x7f, 0x00,
	
	0xda, 0x09,
	
	0xe5, 0x1f,
	0xe1, 0x67,
	0xe0, 0x00,
	0xdd, 0x7f,
	0x05, 0x00,
};
const u8 ov2640_yuv422_reg_tbl[][2]= 
{
	0xFF, 0x00,//regsiter bank select
	0xDA, 0x10,		// image_mode: jpeg
	0xD7, 0x03,		//RESV
	0xDF, 0x00,		// RESV
	0x44, 0x03,		// Quantization
	0x33, 0x80,		
	0x3C, 0x40,
	0xe1, 0x77,
	0x00, 0x00,
};

const u8 ov2640_jpeg_reg_tbl[][2]=
{
	0xff, 0x01, 
	0xe0, 0x14,
	0xe1, 0x67,
	0xe5, 0x1f,
	0xd7, 0x03,
	0xda, 0x10,
	0xe0, 0x00,
};

volatile uint8_t jpeg_data_ok=0;
volatile uint32_t jpeg_data_len=0;
__align(4) uint32_t dcmi_image_buffer_32bit_1[jpeg_buf_size];
namespace devices
{
	extern "C" void DMA2_Stream1_IRQHandler(void)
	{
		/* transfer completed */
		if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
		{
			DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		}

		/* transfer half completed */
		if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_HTIF1) != RESET)
		{
			DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_HTIF1);
		}
	}
	extern "C" void DCMI_IRQHandler(void)
	{
		if (DCMI_GetITStatus(DCMI_IT_FRAME) == SET)
		{
			DCMI_ClearITPendingBit(DCMI_IT_FRAME);
			jpeg_data_process();
		}
	}
	extern "C" int jpeg_data_process()
	{
		if(jpeg_data_ok == Capturing)
		{	
			GPIO_ToggleBits(GPIOD, GPIO_Pin_11);
			DMA_Cmd(DMA2_Stream1, DISABLE);
			while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}
			jpeg_data_len=jpeg_buf_size-DMA_GetCurrDataCounter(DMA2_Stream1);
				
			jpeg_data_ok = Capture_completed;
		}
		if(jpeg_data_ok == Capture_request)
		{
			DMA2_Stream1->NDTR=jpeg_buf_size;	
			DMA_SetCurrDataCounter(DMA2_Stream1,jpeg_buf_size);
			DMA_Cmd(DMA2_Stream1, ENABLE);
			jpeg_data_ok = Capturing;
		}
		return 0;
	}
	
	
	OV2640::OV2640()
	{
		this->_i2c = NULL;
		healthy_flag = false;
	}
	int OV2640::set_out_size(uint16_t width,uint16_t height)
	{
		uint16_t outh;
		uint16_t outw;
		uint8_t temp; 
		if(width%4)
			return 1;
		if(height%4)
			return 2;
		if(_i2c)
		{
			outw=width/4;
			outh=height/4; 
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0XFF,0X00));
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0XE0,0X04));
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X5A,outw&0XFF));
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X5B,outh&0XFF));
			temp=(outw>>8)&0X03;
			temp|=(outh>>6)&0X04;
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X5C,temp));
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0XE0,0X00));
			return 0;
		}
		return -1;
	}
	int OV2640::set_win(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height)
	{
		if(_i2c)
		{
			uint16_t endx;
			uint16_t endy;
			uint8_t temp; 
			endx=offx+width/2;
			endy=offy+height/2;
			
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0Xff,0X01));			
			_i2c->read_reg(OV2640_DEVICE_READ_ADDRESS,0X03,&temp);
			temp&=0XF0;
			temp|=((endy&0X03)<<2)|(offy&0X03);
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X03,temp));				//����Vref��start��end�����2λ
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X19,offy>>2));			//����Vref��start��8λ
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X1A,endy>>2));			//����Vref��end�ĸ�8λ
			
			_i2c->read_reg(OV2640_DEVICE_READ_ADDRESS,0X32,&temp);
			temp&=0Xc0;
			temp|=((endx&0X07)<<3)|(offx&0X07);
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X32,temp));				//����Href��start��end�����3λ
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X17,offx>>3));			//����Href��start��8λ
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0X18,endx>>3));
			return 0;
		}
		return -1;
	}
	int OV2640::init_clock()
	{
		dcmi_clock_init();
		return 0;
	}
	int OV2640::set_format(camera_format format)
	{
		if(!_i2c)
			return -1;
		reset();
		systimer->delayms(200);
		//initialize OV2640
		for(int i=0;i<(sizeof(OV2640_JPEG_INIT)/2);i++)
		{
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_JPEG_INIT[i][0], OV2640_JPEG_INIT[i][1]));
			systimer->delayms(1);
		}
		//set to output YUV422
		for(int i=0;i<(sizeof(OV2640_YUV422)/2);i++)
		{
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_YUV422[i][0], OV2640_YUV422[i][1]));
			systimer->delayms(1);
		}
		FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0xff,0x01));
		FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,0x15,0x00));
		//set to output JPEG
		for(int i=0;i<(sizeof(OV2640_JPEG)/2);i++)
		{
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_JPEG[i][0],OV2640_JPEG[i][1]));
			systimer->delayms(1);
		}
		systimer->delayms(100);
		switch(format)
		{
			case JPEG_160x120:
				
				break;
			case JPEG_176x144:
				
				break;
			case JPEG_320x240:
				for(int i=0;i<(sizeof(OV2640_320x240_JPEG)/2);i++)
				{
					FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_320x240_JPEG[i][0],OV2640_320x240_JPEG[i][1]));
					systimer->delayms(1);
				}
				break;
			case JPEG_352x288:
				for(int i=0;i<(sizeof(OV2640_352x288_JPEG)/2);i++)
				{
					FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_352x288_JPEG[i][0],OV2640_352x288_JPEG[i][1]));
					systimer->delayms(1);
				}
				break;
			case JPEG_640x480:
				for(int i=0;i<(sizeof(OV2640_640x480_JPEG)/2);i++)
				{
					FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_640x480_JPEG[i][0],OV2640_640x480_JPEG[i][1]));
					systimer->delayms(1);
				}
				break;
			default:
				
				break;
		}
		return 0;
	}
	int OV2640::sxga_mode()
	{
		if(_i2c)
		{
			for(int i=0;i<(sizeof(ov2640_sxga_init_reg_tbl)/2);i++)
			{
				FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,ov2640_sxga_init_reg_tbl[i][0], ov2640_sxga_init_reg_tbl[i][1]));
				systimer->delayms(1);
			}
			systimer->delayms(200);
			return 0;
		}
		return -1;
	}
	int OV2640::svga_mode()
	{
		if(_i2c)
		{
			//set svga mode
			for(int i=0;i<(sizeof(ov2640_svga_init_reg_tbl)/2);i++)
			{
				FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,ov2640_svga_init_reg_tbl[i][0], ov2640_svga_init_reg_tbl[i][1]));
				systimer->delayms(1);
			}
			return 0;
		}
		return -1;
	}
	int OV2640::jpeg_mode()
	{
		if(_i2c)
		{
			//set YUV422 format
			for(int i=0;i<(sizeof(ov2640_yuv422_reg_tbl)/2);i++)
			{
				FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,ov2640_yuv422_reg_tbl[i][0], ov2640_yuv422_reg_tbl[i][1]));
				systimer->delayms(1);
			}
			//output jpeg data
			for(int i=0;i<(sizeof(ov2640_jpeg_reg_tbl)/2);i++)
			{
				FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_READ_ADDRESS,ov2640_jpeg_reg_tbl[i][0],ov2640_jpeg_reg_tbl[i][1]));
				systimer->delayms(1);
			}
			return 0;
		}
		return -1;
	}
	int OV2640::enable_capture()
	{
		dcmi_dma_enable();
		return 0;
	}
	int OV2640::disable_capture()
	{
		dcmi_dma_disable();
		return 0;
	}
	int OV2640::reset()
	{
		if(_i2c)
		{
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_WRITE_ADDRESS,OV2640_DSP_RA_DLMT,0x01));
			FAIL_RETURN(_i2c->write_reg(OV2640_DEVICE_WRITE_ADDRESS,OV2640_SENSOR_COM7,0x80));
		}
		return 0;
	}
	bool OV2640::is_frame_ready()
	{
		if(jpeg_data_ok == Capture_completed)
		{
			return true;
		}
		return false;
	}
	void OV2640::capture_request()
	{
		jpeg_data_ok = Capture_request;
	}
	bool OV2640::healthy()
	{
		return healthy_flag;
	}
	int OV2640::get_current_frame_len()
	{
		int index = 0;
		uint8_t *pbuf = (uint8_t *)dcmi_image_buffer_32bit_1;
		for(index=0;index<jpeg_data_len*4;index++)
		{
			if((pbuf[index]==0XFF)&&(pbuf[index+1]==0XD8))
				break;
		}
		if(index == jpeg_data_len*4)
			return -1;
		return jpeg_data_len*4 - index;
	}
	uint8_t *OV2640::get_data_ptr()
	{
		int index = 0;
		uint8_t *pbuf = (uint8_t *)dcmi_image_buffer_32bit_1;
		for(index=0;index<jpeg_data_len*4;index++)
		{
			if((pbuf[index]==0XFF)&&(pbuf[index+1]==0XD8))
				break;
		}
		if(index == jpeg_data_len*4)
			return NULL;
		return pbuf+index;
	}	
	
	int OV2640::init(II2C *i2c)
	{
		uint8_t Mid_H = 0;
		uint8_t Mid_L = 0;
		int ret = 0;
		if(!i2c)
			return -1;
		_i2c = i2c;
		systimer->delayms(100);
		reset();
		ret = _i2c->read_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_SENSOR_MIDH,&Mid_H);
		ret = _i2c->read_reg(OV2640_DEVICE_READ_ADDRESS,OV2640_SENSOR_MIDL,&Mid_L);
		if(Mid_H != OV2640_MIDH_VALUE || Mid_L != OV2640_MIDL_VALUE)
		{
			return -10;
		}

		dcmi_hw_init();
		dcmi_dma_init((uint32_t)dcmi_image_buffer_32bit_1,jpeg_buf_size,JPEG_320x240);

		//ret = svga_mode();
		ret = sxga_mode();
		if(ret < 0)
			return -1;
		ret = jpeg_mode();
		if(ret < 0) 
			return -2;
		ret = set_out_size(1600,1200);
		if(ret < 0)
			return -4;
		dcmi_dma_enable();
		healthy_flag = true;

		return 0;//
	}
}