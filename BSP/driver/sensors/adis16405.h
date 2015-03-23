#pragma once

#ifdef __cplusplus
extern "C" {
#endif
	
#include "../common/mcu.h"

typedef struct
{
	unsigned short supply_measurement;
	signed short gyro_x;
	signed short gyro_y;
	signed short gyro_z;
	signed short accel_x;
	signed short accel_y;
	signed short accel_z;
	signed short mag_x;
	signed short mag_y;
	signed short mag_z;
	unsigned short temperature;
	unsigned short aux_adc;
} adis16405_burst_packet;

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

int adis16405_init(void);
int adis16405_read_register(unsigned char registerAddress, unsigned short *out);
int adis16405_write_register(unsigned char registerAddress, unsigned short new_value);
int adis16405_burst_read(adis16405_burst_packet *out);
int16_t ReadFromADIS16405ViaSpi(unsigned char RegisterAddress);
void ReadFromADIS16405ViaSpi2(unsigned char RegisterAddress, unsigned char NumberofRegisters, unsigned char *RegisterData);
static void translate_14bit(signed short *data)
{
	unsigned short *p = (unsigned short *)data;
	if (*p & 0x2000)
		*p |= 0xE000;
	else
		*p &= 0x1FFF;
}


#ifdef __cplusplus
}
#endif
