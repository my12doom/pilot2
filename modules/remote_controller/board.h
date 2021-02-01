#pragma once

#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>
#include <HAL/Interface/IInterrupt.h>
#include <HAL/Interface/ITimer.h>
#include <HAL/Interface/IRCOUT.h>
#include <HAL/Interface/IUART.h>
#include <string.h>
#include <Protocol/crc32.h>

extern HAL::ISPI *spi;
extern HAL::IGPIO *cs;
extern HAL::IGPIO *ce;
extern HAL::IGPIO *irq;
extern HAL::IGPIO *dbg;
extern HAL::IGPIO *dbg2;
extern HAL::IInterrupt *interrupt;
extern HAL::ITimer *timer;
extern HAL::IGPIO *bind_button;

extern HAL::IGPIO *SCL;
extern HAL::IGPIO *SDA;
extern HAL::IRCOUT *ppm;
extern HAL::IUART *uart;
extern HAL::IGPIO *vibrator;

typedef struct 
{
	unsigned _min:12;
	unsigned _max:12;
	unsigned middle:12;
	unsigned dead_band:7;
	unsigned reverse:1;
} configure_entry;

int board_init();
void read_channels(int16_t *channel, int max_channel_count);
void select_ant(uint32_t *randomizer, bool tx);		// 16bytes randomizer
void read_keys(uint8_t* key, int max_keys);
void custom_output(uint8_t * payload, int payload_size, int latency);

static uint64_t board_get_seed()
{	
	/*
		Device line	Starting address
		F0, F3	0x1FFFF7AC
		F1	0x1FFFF7E8
		F2, F4	0x1FFF7A10
		F7	0x1FF0F420
		L0	0x1FF80050
		L0, L1 Cat.1,Cat.2	0x1FF80050
		L1 Cat.3,Cat.4,Cat.5,Cat.6	0x1FF800D0
	*/

	#ifdef STM32F0XX
	const void *stm32_id_address = (const void*)0x1FFFF7AC;	
	#endif
	
	#if (defined (STM32F10X_LD) || defined(STM32F10X_MD) || defined (STM32F10X_MD_VL))
	const void *stm32_id_address = (const void*)0x1FFFF7E8;	
	#endif
	
	char data[12];
	memcpy(data, stm32_id_address, 12);

	uint64_t o;	
	((uint32_t*)&o)[0] = crc32(0, data, 8);
	((uint32_t*)&o)[1] = crc32(0, data+4, 8);
	
	// TODO: add true random number
	
	return o;
}

