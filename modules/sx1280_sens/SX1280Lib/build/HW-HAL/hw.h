#ifndef __HW_H__
#define __HW_H__

#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include "hw-spi.h"
#include "hw-uart.h"
#include "hw-gpio.h"

#include "sx1280.h"
#include "sx1280-hal.h"

#define BOARD_NUCLEO_L476RG
#define SHIELD_PCB_E394V02A

#include "boards/boards.h"

#define USE_DMA

void HwInit( void );

void HwSetLowPower( void );

void SystemClock_Config( void );

void HAL_Delay( uint32_t Delay );

void Error_Handler( void );

#endif // __HW_H__
