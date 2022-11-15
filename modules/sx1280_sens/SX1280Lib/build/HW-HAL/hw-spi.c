#include "hw.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef SpiHandle;
volatile bool blockingDmaFlag;

void SpiInit( void )
{
    SpiHandle.Instance                = SPI1;
    SpiHandle.Init.Mode               = SPI_MODE_MASTER;
    SpiHandle.Init.Direction          = SPI_DIRECTION_2LINES;
    SpiHandle.Init.DataSize           = SPI_DATASIZE_8BIT;
    SpiHandle.Init.CLKPolarity        = SPI_POLARITY_LOW;
    SpiHandle.Init.CLKPhase           = SPI_PHASE_1EDGE;
    SpiHandle.Init.NSS                = SPI_NSS_SOFT;
    SpiHandle.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_16;
    SpiHandle.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode             = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial      = 7;
    SpiHandle.Init.CRCLength          = SPI_CRC_LENGTH_DATASIZE;
    SpiHandle.Init.NSSPMode           = SPI_NSS_PULSE_DISABLE;

    if ( HAL_SPI_Init( &SpiHandle ) != HAL_OK )
    {
        Error_Handler( );
    }
}

void SpiDeInit( void )
{
    HAL_SPI_DeInit( &SpiHandle );
}

#define WAIT_FOR_BLOCKING_FLAG         while( blockingDmaFlag ) { }
/*!
 * @brief Sends txBuffer and receives rxBuffer
 *
 * @param [IN] txBuffer Byte to be sent
 * @param [OUT] rxBuffer Byte to be sent
 * @param [IN] size Byte to be sent
 */
void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{
    HAL_SPIEx_FlushRxFifo( &SpiHandle );
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_TransmitReceive_DMA( &SpiHandle, txBuffer, rxBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
    #else
        HAL_SPI_TransmitReceive( &SpiHandle, txBuffer, rxBuffer, size, HAL_MAX_DELAY );
    #endif
}

void SpiIn( uint8_t *txBuffer, uint16_t size )
{
    #ifdef USE_DMA
        blockingDmaFlag = true;
        HAL_SPI_Transmit_DMA( &SpiHandle, txBuffer, size );
        WAIT_FOR_BLOCKING_FLAG
    #else
        HAL_SPI_Transmit( &SpiHandle, txBuffer, size, HAL_MAX_DELAY );
    #endif
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    blockingDmaFlag = false;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    blockingDmaFlag = false;
}
