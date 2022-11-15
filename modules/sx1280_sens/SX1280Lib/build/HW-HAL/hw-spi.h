#ifndef __HW_SPI_H__
#define __HW_SPI_H__

#include <stdint.h>

void SpiInit( void );

void SpiDeInit( void );

void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size );

void SpiIn( uint8_t *txBuffer, uint16_t size );

#endif // __HW_SPI_H__
