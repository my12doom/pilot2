#pragma once

#include <stdint.h>

#define NEWSCREEN

extern uint16_t CB;
extern uint16_t CF;

//定义几个常用颜色
#define RED 0x00F8
#define ORA 0x00FD
#define YEL 0xE0FF
#define GRE 0xE007
#define CYA 0xFF07
#define BLU 0x1F00
#define MAG 0x1FF8
#define BLA 0x0000
#define WHI 0xFFFF

//SPI
void SPIWx8(uint16_t dat);
void SPIWdot(uint16_t dat);
void SPIWDAT(uint8_t dat);
void SPIW2DAT(uint16_t dat);
void SPIWCMD(uint8_t dat);
void SPIWx8(uint16_t dat);
void Delay(volatile uint32_t i);//每次循环i-1直到i=0退出,__IO则不优化这句或这个变量


#define LEDH	GPIOA->BSRR = GPIO_Pin_9//A9接背光LED，高电平
#define LEDL	GPIOA->BRR  = GPIO_Pin_9//A9接背光LED，低电平

//#define CL(A5)硬件端口不定义，知道是接这里就行
#define RS_L				(GPIOA->BRR = GPIO_Pin_6)
#define RS_H				(GPIOA->BSRR = GPIO_Pin_6)

//#define DA(A7)硬件端口不定义，知道是接这里就行
#define RST_L				(GPIOB->BRR = GPIO_Pin_1)
#define RST_H				(GPIOB->BSRR = GPIO_Pin_1)


uint8_t RND(void);//随机数生成器0-15随机数
void GPIOConfig(void);//配置端口
void PInit(void);//屏幕初始化
void PBox(uint32_t POS);//划定刷屏窗口,32位代表X0 Y0 X1 Y1 左上角为0点
void PSquare(uint32_t POS);//画方框填充CB色，可用于清屏
void PLogo(void);//显示256色的开机画面
void PString(char *p,uint16_t POS);//显示字符串，位置X0Y0
void PNum(uint32_t Num,uint16_t POS);//显示5位整数，位置X0Y0
void PHex(uint8_t Num,uint16_t POS);//显示一个16进制数字，位置X0Y0
void ERR(char *p);//显示错误信息


