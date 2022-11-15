/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Matthieu Verdy and Benjamin Boulet
*/
#include "sx1280-hal.h"

#include <string.h>

#include "sx1280.h"

/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE   0xFFF

#define IRQ_HIGH_PRIORITY  0


/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
void SX1280HalWaitOnBusy( SX1280_t *sx1280 )
{
    while( sx1280->get_busy( sx1280->ctx ) == 1 );
}

void SX1280HalInit( SX1280_t *sx1280, DioIrqHandler **irqHandlers )
{
    SX1280HalReset( sx1280 );
    SX1280HalIoIrqInit( sx1280, irqHandlers );
}

// TODO: this needs to be replaced w/ something workable
void SX1280HalIoIrqInit( SX1280_t *sx1280, DioIrqHandler **irqHandlers )
{
#if 0

#if( RADIO_DIO1_ENABLE )
    GpioSetIrq( RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0] );
#endif
#if( RADIO_DIO2_ENABLE )
	GpioSetIrq( RADIO_DIO2_GPIO_Port, RADIO_DIO2_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0] );
#endif
#if( RADIO_DIO3_ENABLE )
	GpioSetIrq( RADIO_DIO3_GPIO_Port, RADIO_DIO3_Pin, IRQ_HIGH_PRIORITY, irqHandlers[0] );
#endif
#if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
#error "Please define a DIO" 
#endif

#endif
}

void SX1280HalReset( SX1280_t *sx1280 )
{
    sx1280->delay_ms( sx1280->ctx, 20 );
    sx1280->set_reset( sx1280->ctx, 0 );
    sx1280->delay_ms( sx1280->ctx, 50 );
    sx1280->set_reset( sx1280->ctx, 1 );
    sx1280->delay_ms( sx1280->ctx, 20 );
}

void SX1280HalClearInstructionRam( SX1280_t *sx1280 )
{
    // Clearing the instruction RAM is writing 0x00s on every bytes of the
    // instruction RAM
    uint8_t cmd_out[3];
    cmd_out[0] = RADIO_WRITE_REGISTER;
    cmd_out[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
    cmd_out[2] = IRAM_START_ADDRESS & 0x00FF;

    uint8_t iram[IRAM_SIZE];
    memset(iram, 0x00, IRAM_SIZE);

    SX1280HalWaitOnBusy( sx1280 );

    sx1280->spi_write(sx1280->ctx, cmd_out, sizeof(cmd_out), iram, sizeof(iram));

    SX1280HalWaitOnBusy( sx1280 );
}

void SX1280HalWakeup( SX1280_t *sx1280 )
{
    // TODO: disable HAL IRQ handling rather than globally
    //__disable_irq( );

    uint8_t cmd_out[2];
    cmd_out[0] = RADIO_GET_STATUS;
    cmd_out[1] = 0x00;
    
    sx1280->spi_write(sx1280->ctx, cmd_out, sizeof(cmd_out), NULL, 0);
    
    // Wait for chip to be ready.
    SX1280HalWaitOnBusy( sx1280 );

    // TODO: disable HAL IRQ handling rather than globally
    //__enable_irq( );
}

void SX1280HalWriteCommand( SX1280_t *sx1280, uint8_t command, uint8_t *buffer, uint16_t size )
{
    SX1280HalWaitOnBusy( sx1280 );

    uint8_t cmd_out[1];
    cmd_out[0] = command;

    sx1280->spi_write(sx1280->ctx, cmd_out, sizeof(cmd_out), buffer, size);

    if( command != RADIO_SET_SLEEP )
    {
        SX1280HalWaitOnBusy( sx1280 );
    }
}

void SX1280HalReadCommand( SX1280_t *sx1280, uint8_t command, uint8_t *buffer, uint16_t size )
{
    uint8_t cmd_out[2];
    cmd_out[0] = command;
    cmd_out[1] = 0x00;

    SX1280HalWaitOnBusy( sx1280 );

    sx1280->spi_read(sx1280->ctx, cmd_out, sizeof(cmd_out), buffer, size);

    SX1280HalWaitOnBusy( sx1280 );
}

void SX1280HalWriteRegisters( SX1280_t *sx1280, uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint8_t cmd_out[3];
    cmd_out[0] = RADIO_WRITE_REGISTER;
    cmd_out[1] = ( address & 0xFF00 ) >> 8;
    cmd_out[2] = address & 0x00FF;

    SX1280HalWaitOnBusy( sx1280 );

    sx1280->spi_write(sx1280->ctx, cmd_out, sizeof(cmd_out), buffer, size);

    SX1280HalWaitOnBusy( sx1280 );
}

void SX1280HalWriteRegister( SX1280_t *sx1280, uint16_t address, uint8_t value )
{
    SX1280HalWriteRegisters( sx1280, address, &value, 1 );
}

void SX1280HalReadRegisters( SX1280_t *sx1280, uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint8_t cmd_out[4];
    cmd_out[0] = RADIO_READ_REGISTER;
    cmd_out[1] = ( address & 0xFF00 ) >> 8;
    cmd_out[2] = address & 0x00FF;
    cmd_out[3] = 0x00;

    memset(buffer, 0x00, size);

    SX1280HalWaitOnBusy( sx1280 );

    sx1280->spi_read(sx1280->ctx, cmd_out, sizeof(cmd_out), buffer, size);

    SX1280HalWaitOnBusy( sx1280 );
}

uint8_t SX1280HalReadRegister( SX1280_t *sx1280, uint16_t address )
{
    uint8_t data;

    SX1280HalReadRegisters( sx1280, address, &data, 1 );

    return data;
}

void SX1280HalWriteBuffer( SX1280_t *sx1280, uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint8_t cmd_out[2];
    cmd_out[0] = RADIO_WRITE_BUFFER;
    cmd_out[1] = offset;

    SX1280HalWaitOnBusy( sx1280 );

    sx1280->spi_write(sx1280->ctx, cmd_out, sizeof(cmd_out), buffer, size);

    SX1280HalWaitOnBusy( sx1280 );
}

void SX1280HalReadBuffer( SX1280_t *sx1280, uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint8_t cmd_out[3];
    cmd_out[0] = RADIO_READ_BUFFER;
    cmd_out[1] = offset;
    cmd_out[2] = 0x00;

    memset(buffer, 0x00, size);

    sx1280->spi_read(sx1280->ctx, cmd_out, sizeof(cmd_out), buffer, size);

    SX1280HalWaitOnBusy( sx1280 );
}

uint8_t SX1280HalGetDioStatus( SX1280_t *sx1280 )
{
	uint8_t Status = sx1280->get_busy( sx1280->ctx );
	
#if( RADIO_DIO1_ENABLE )
	Status |= (sx1280->get_dio[0](sx1280->ctx) << 1);
#endif
#if( RADIO_DIO2_ENABLE )
	Status |= (sx1280->get_dio[1](sx1280->ctx) << 2);
#endif
#if( RADIO_DIO3_ENABLE )
	Status |= (sx1280->get_dio[2](sx1280->ctx) << 3);
#endif
#if( !RADIO_DIO1_ENABLE && !RADIO_DIO2_ENABLE && !RADIO_DIO3_ENABLE )
#error "Please define a DIO" 
#endif
	
	return Status;
}
