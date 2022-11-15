#ifndef __HW_GPIO_H__
#define __HW_GPIO_H__


typedef void( GpioIrqHandler )( void );

/*!
 * GPIO IRQ handler function prototype
 */
IRQn_Type MSP_GetIRQn( uint16_t gpioPin );

void GpioInit( void );

void GpioDeInit( void );

void GpioWrite( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,  uint32_t value );

uint32_t GpioRead( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );

uint8_t GpioGetBitPos( uint16_t GPIO_Pin );

void GpioSetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler );

void GpioLaunchIrqHandler( uint16_t GPIO_Pin );

#endif // __HW_GPIO_H__
