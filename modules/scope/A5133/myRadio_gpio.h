
#ifndef __MYRADIO_GPIO_H_
#define __MYRADIO_GPIO_H_

#include <stdint.h>
#include <stdbool.h>

typedef void (*RADIO_GPIO_CALLBACK)(uint8_t index);


uint8_t READ_RF_A5133_GPIO1(void);
//void RF_A5133_GPIO1_H(void);
//void RF_A5133_GPIO1_L(void);

void BOARD_A5133_SCS_H(void);
void BOARD_A5133_SCS_L(void);

//void SET_A5133_GPIO1_IN(void);
//void SET_A5133_GPIO1_OUT(void);
//void BOARD_A5133_SDIO_H(void);
//void BOARD_A5133_SDIO_L(void);
//void BOARD_A5133_SCK_H(void);
//void BOARD_A5133_SCK_L(void);

void RF_EXT_PA_TO_TX(void);
void RF_EXT_PA_TO_RX(void);
void RF_EXT_PA_TO_IDLE(void);

void tst();

//-------------将封装的API映射到射频模块硬件层---------------
void myRadio_gpio_init(RADIO_GPIO_CALLBACK cb);
//void myRadio_gpio_intoLwPwr(bool sta);
uint8_t myRadioSpi_rByte(void);
void myRadioSpi_wByte(uint8_t byteToWrite);

void myRadioSpi_trx(uint8_t *tx, uint8_t *rx, int count);
//-------------将封装的API映射到射频模块硬件层---------------END
#endif
