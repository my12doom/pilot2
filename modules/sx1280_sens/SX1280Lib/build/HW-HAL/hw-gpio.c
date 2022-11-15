
#include "hw.h"


static GpioIrqHandler *GpioIrq[16] = { NULL };


void GpioInit( void )
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE( );
    __HAL_RCC_GPIOB_CLK_ENABLE( );
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    __HAL_RCC_GPIOD_CLK_ENABLE( );
    __HAL_RCC_GPIOE_CLK_ENABLE( );
    __HAL_RCC_GPIOF_CLK_ENABLE( );
    __HAL_RCC_GPIOG_CLK_ENABLE( );
    __HAL_RCC_GPIOH_CLK_ENABLE( );

    /*Configure GPIO pins : PC13 PC2 PC3 PC4 
                           PC5 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 
                          | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 
                          | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    /*Configure GPIO pins : LED_RX_Pin LED_TX_Pin */
    GPIO_InitStruct.Pin = LED_RX_PIN | LED_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    /*Configure GPIO pins : nRESET_Pin RADIO_NSS_Pin */
    GPIO_InitStruct.Pin = RADIO_nRESET_PIN | RADIO_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    /*Configure GPIO pins : PA1 PA4 PA9 PA10 
                           PA11 PA12 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_9 | GPIO_PIN_10 
                          | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    /*Configure GPIO pin : ANT_SW_Pin */
    GPIO_InitStruct.Pin = ANT_SW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ANT_SW_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : PB1 PB2 PB10 PB11 
                           PB12 PB13 PB14 PB15 
                           PB5 PB6 PB7 PB8 
                           PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 
                          | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 
                          | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 
                          | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

    /*Configure GPIO pin : PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOD, &GPIO_InitStruct );

    /*Configure GPIO pin : BUSY_Pin */
    GPIO_InitStruct.Pin = RADIO_BUSY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init( RADIO_BUSY_PORT, &GPIO_InitStruct );

    /*Configure GPIO pin : DIOx_Pin */
    GPIO_InitStruct.Pin = RADIO_DIOx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init( RADIO_DIOx_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    GpioWrite( GPIOC, LED_RX_PIN | LED_TX_PIN, GPIO_PIN_SET );

    /*Configure GPIO pin Output Level */
    GpioWrite( GPIOA, RADIO_nRESET_PIN | RADIO_NSS_PIN, GPIO_PIN_RESET );

    /*Configure GPIO pin Output Level */
    GpioWrite( ANT_SW_PORT, ANT_SW_PIN, GPIO_PIN_RESET );

}


void GpioDeInit( void )
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pins : USART_TX_PIN */
    GPIO_InitStruct.Pin = USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( USART_TX_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : USART_RX_PIN */
    GPIO_InitStruct.Pin = USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( USART_RX_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : LED_RX_Pin */
    GPIO_InitStruct.Pin = LED_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( LED_RX_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : LED_TX_PIN */
    GPIO_InitStruct.Pin = LED_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( LED_TX_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : nRESET_Pin */
    GPIO_InitStruct.Pin = RADIO_nRESET_PIN ;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( RADIO_nRESET_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : RADIO_NSS_Pin */
    GPIO_InitStruct.Pin = RADIO_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( RADIO_NSS_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : RADIO_MOSI_PIN */
    GPIO_InitStruct.Pin = RADIO_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( RADIO_MOSI_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : RADIO_MISO_PIN */
    GPIO_InitStruct.Pin = RADIO_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( RADIO_MISO_PORT, &GPIO_InitStruct );

    /*Configure GPIO pins : RADIO_SCK_PIN */
    GPIO_InitStruct.Pin = RADIO_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( RADIO_SCK_PORT, &GPIO_InitStruct );

    /*Configure GPIO pin : ANT_SW_Pin */
    GPIO_InitStruct.Pin = ANT_SW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ANT_SW_PORT, &GPIO_InitStruct );

    /*Configure GPIO pin : PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOD, &GPIO_InitStruct );

    /*Configure GPIO pin : BUSY_Pin */
    GPIO_InitStruct.Pin = RADIO_BUSY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( RADIO_BUSY_PORT, &GPIO_InitStruct );

    /*Configure GPIO pin : DIOx_Pin */
    GPIO_InitStruct.Pin = RADIO_DIOx_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( RADIO_DIOx_PORT, &GPIO_InitStruct);



    /*Configure GPIO pins : PA1 PA4 PA9 PA10 
                           PA11 PA12 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_9 | GPIO_PIN_10 
                          | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );
    
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOD, &GPIO_InitStruct );
    
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOE, &GPIO_InitStruct );
    
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOF, &GPIO_InitStruct );
    
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOH, &GPIO_InitStruct );
    
    
      /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE( );
    __HAL_RCC_GPIOB_CLK_DISABLE( );
    __HAL_RCC_GPIOC_CLK_DISABLE( );
    __HAL_RCC_GPIOD_CLK_DISABLE( );
    __HAL_RCC_GPIOE_CLK_DISABLE( );
    __HAL_RCC_GPIOF_CLK_DISABLE( );
    __HAL_RCC_GPIOG_CLK_DISABLE( );
    __HAL_RCC_GPIOH_CLK_DISABLE( );
}



/*!
 * @brief Records the interrupt handler for the GPIO  object
 *
 * @param  GPIOx: where x can be (A..E and H) 
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] prio       NVIC priority (0 is highest)
 * @param [IN] irqHandler  points to the  function to execute
 * @retval none
 */
void GpioSetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler )
{
    IRQn_Type IRQnb;

    uint32_t BitPos = GpioGetBitPos( GPIO_Pin ) ;

    if ( irqHandler != NULL )
    {
        GpioIrq[BitPos] = irqHandler;

        IRQnb = MSP_GetIRQn( GPIO_Pin );

        HAL_NVIC_SetPriority( IRQnb , prio, 0 );

        HAL_NVIC_EnableIRQ( IRQnb );
    }
}

/*!
 * @brief Execute the interrupt from the object
 *
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval none
 */
void GpioLaunchIrqHandler( uint16_t GPIO_Pin )
{
    uint32_t BitPos = GpioGetBitPos( GPIO_Pin );

    if ( GpioIrq[BitPos]  != NULL )
    {
        GpioIrq[BitPos]( );
    }
}


/*!
 * @brief Get the position of the bit set in the GPIO_Pin
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval the position of the bit
 */
uint8_t GpioGetBitPos( uint16_t GPIO_Pin )
{
    uint8_t PinPos=0;

    if ( ( GPIO_Pin & 0xFF00 ) != 0 ) { PinPos |= 0x8; }
    if ( ( GPIO_Pin & 0xF0F0 ) != 0 ) { PinPos |= 0x4; }
    if ( ( GPIO_Pin & 0xCCCC ) != 0 ) { PinPos |= 0x2; }
    if ( ( GPIO_Pin & 0xAAAA ) != 0 ) { PinPos |= 0x1; }

    return PinPos;
}


/*!
 * @brief Writes the given value to the GPIO output
 *
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @param [IN] value New GPIO output value
 * @retval none
 */
void GpioWrite( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t value )
{
    HAL_GPIO_WritePin( GPIOx, GPIO_Pin , ( GPIO_PinState ) value );
}

/*!
 * @brief Reads the current GPIO input value
 *
 * @param  GPIOx: where x can be (A..E and H) 
 * @param  GPIO_Pin: specifies the port bit to be written.
 *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
 *                   All port bits are not necessarily available on all GPIOs.
 * @retval value   Current GPIO input value
 */
uint32_t GpioRead( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
    return HAL_GPIO_ReadPin( GPIOx, GPIO_Pin );
}
