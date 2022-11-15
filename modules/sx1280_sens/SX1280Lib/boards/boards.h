#ifndef __BOARDS_H__
#define __BOARDS_H__

#include <stm32F4xx_gpio.h>

#define RADIO_NSS_PIN       GPIO_Pin_4
#define RADIO_NSS_PORT      GPIOA

#define ANT_SW_PIN          GPIO_Pin_0
#define ANT_SW_PORT         GPIOB

#define RADIO_nRESET_PIN    GPIO_Pin_2
#define RADIO_nRESET_PORT   GPIOB

#define RADIO_MOSI_PIN      GPIO_Pin_7
#define RADIO_MOSI_PORT     GPIOA

#define RADIO_MISO_PIN      GPIO_Pin_6
#define RADIO_MISO_PORT     GPIOA

#define RADIO_SCK_PIN       GPIO_Pin_5
#define RADIO_SCK_PORT      GPIOA

#define RADIO_BUSY_PIN      GPIO_Pin_11
#define RADIO_BUSY_PORT     GPIOA

//#define RADIO_DIOx_PIN      GPIO_Pin_1
//#define RADIO_DIOx_PORT     GPIOA

//#define USART_TX_PIN        GPIO_Pin_2
//#define USART_TX_PORT       GPIOA

//#define USART_RX_PIN        GPIO_Pin_3
//#define USART_RX_PORT       GPIOA

//#define LED_RX_PIN          GPIO_Pin_0
//#define LED_RX_PORT         GPIOC

//#define LED_TX_PIN          GPIO_Pin_1
//#define LED_TX_PORT         GPIOC

#define RADIO_DIO1_Pin      GPIO_Pin_1
#define RADIO_DIO1_GPIO_Port    GPIOA

#endif // __BOARDS_H__
