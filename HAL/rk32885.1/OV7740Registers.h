#ifndef _OV7740REGISTERS_H
#define _OV7740REGISTERS_H


#define OV7740_SLAVE_ADDR       0x42U                     
#define OV7740_SLAVE_ADDR2      0x42U
#define OV7740_SLAVE_AF_ADDR    0x18U 

/*
* set AEC_TIME_H to 0x00 ,AEC_TIME_L set to 0x70
*/
#define AEC_CTL 				0x13
#define AEC_TIME_H				0x0f
#define AEC_TIME_L				0x10

#define YAVG					0x2f
/*
*	WPT high threshold value ,BPT low threshold value
*/
#define WPT 					0x24
#define BPT						0x25
#define AGC_CTL_H				0x15
#define AGC_CTL_L				0x00

/*
* typical usage settings
*/
#define AEC_AGC_CLOSE 0x82
#define AGC_OPEN 	  0x86
#endif
