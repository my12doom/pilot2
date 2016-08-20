#pragma once

#include <stdint.h>

typedef enum
{
	ads1256_speed_30000sps = 0xf0,
	ads1256_speed_15000sps = 0xe0,
	ads1256_speed_7500sps = 0xd0,
	ads1256_speed_3750sps = 0xc0,
	ads1256_speed_2000sps = 0xb0,
	ads1256_speed_1000sps = 0xa1,
	ads1256_speed_500sps = 0x92,
	ads1256_speed_100sps = 0x82,
	ads1256_speed_60sps = 0x72,
	ads1256_speed_50sps = 0x63,
	ads1256_speed_30sps = 0x53,
	ads1256_speed_25sps = 0x43,
	ads1256_speed_15sps = 0x33,
	ads1256_speed_10sps = 0x23,
	ads1256_speed_5sps = 0x13,
	ads1256_speed_2_5sps = 0x03,
} ads1256_speed;

typedef enum
{
	ads1256_channnel_AIN0 = 0,
	ads1256_channnel_AIN1 = 1,
	ads1256_channnel_AIN2 = 2,
	ads1256_channnel_AIN3 = 3,
	ads1256_channnel_AIN4 = 4,
	ads1256_channnel_AIN5 = 5,
	ads1256_channnel_AIN6 = 6,
	ads1256_channnel_AIN7 = 7,
	ads1256_channnel_AINCOM = 8,
} ads1256_channel;

typedef enum
{
	ads1256_gain_1 = 0,		// x1
	ads1256_gain_2 = 1,		// x2
	ads1256_gain_4 = 2,		// x4
	ads1256_gain_8 = 3,		// x8
	ads1256_gain_16 = 4,	// x16
	ads1256_gain_32 = 5,	// x32
	ads1256_gain_64 = 6,	// x64
	ads1256_gain_64_2 = 7,	// x64
} ads1256_PGA;


typedef struct
{
	unsigned DataReady:1;
	unsigned BufferEnable:1;
	unsigned AutoCalibration:1;
	unsigned BitOrder:1;
	unsigned ID:4;
} ads1256_status;						// status register(0)

typedef struct
{
	unsigned negative:4;
	unsigned positive:4;
} ads1256_mux;						// input multiplexer register(1)

typedef struct
{
	unsigned PGA:3;
	unsigned SensorDetect:2;
	unsigned ClockOut:2;
	unsigned Reserved:1;
} ads1256_ad_controll;				// A/D Controll register(2)

typedef struct
{
	unsigned STATUS0:1;
	unsigned STATUS1:1;
	unsigned STATUS2:1;
	unsigned STATUS3:1;
	unsigned DIR0:1;
	unsigned DIR1:1;
	unsigned DIR2:1;
	unsigned DIR3:1;
} ads1256_gpio_config;					// gpio config register(4)

typedef struct
{
	unsigned mode0:2;
	unsigned io0:2;
} CRH;


typedef enum
{
	REG_Status = 0,
	REG_MUX = 1,
	REG_ADControll = 2,
	REG_DataRate = 3,
	REG_GPIO = 4,
	REG_OffsetCalibrationLSB = 5,
	REG_OffsetCalibrationB = 6,
	REG_OffsetCalibrationMSB = 7,
	REG_GainCalibrationLSB = 8,
	REG_GainCalibrationB = 9,
	REG_GainCalibrationMSB = 10,
} ads1256_register;

typedef enum
{
	CMD_WakeUp = 0x00,
	CMD_ReadData = 0x01,
	CMD_ReadDataContinuously = 0x03,
	CMD_StopReadDataContinuously = 0x0f,
	//REG_ReadReg = 0x10,					// don't use this command, use ads1256_read_register()
	//REG_WriteReg = 0x50,					// don't use this command, use ads1256_write_register()
	CMD_OffsetGainCalibration = 0xf0,
	CMD_OffsetCalibration = 0xf1,
	CMD_GainCalibration = 0xf2,
	CMD_SystemOffsetCalibration = 0xf3,
	CMD_SystemGainCalibration = 0xf4,
	CMD_Sync = 0xfc,
	CMD_Standby = 0xfd,
	CMD_Reset = 0xfe,
	CMD_WakeUp2 = 0xff,
} ads1256_command;

#ifdef __cplusplus
extern "C" {
#endif

void ads1256_end();
void ads1256_begin();
uint8_t ads1256_tx_rx(uint8_t Data);
uint8_t ads1256_read_registers(uint8_t start, uint8_t n, void *out);
uint8_t ads1256_write_registers(uint8_t start, uint8_t n, void *out);
uint8_t ads1256_read_register(uint8_t reg);
void ads1256_go();
uint8_t ads1256_read_register(uint8_t reg);
void ads1256_write_register(uint8_t reg, uint8_t data);
int ads1256_init(void);
int ads1256_startconvert(void);
int ads1256_getresult(short *result);		// return -1 if still converting, 0 if conversion completed or continuous mode, further calls return the last conversion result.
short ads1256_convert(void);				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly

#ifdef __cplusplus
}
#endif
