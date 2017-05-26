#include "oled.h"
#include <stm32f0xx.h>
#include "ascii.h"

//全局变量声明
uint8_t X0,X1,Y0,Y1;//显示窗口坐标矩形四角
uint16_t CB=BLA,CF=WHI;//背景颜色CB，前景颜色CF
uint16_t ADCDAT[4];//ADC采样数组
uint32_t i,j,k;//临时变量
uint8_t r;


//配置端口-----------------------------------------------------------------
void GPIOConfig(void)//配置端口
{
	GPIO_InitTypeDef		GPIO_InitStruct;//过会儿要玩GPIO所以把这个结构体弄好
	SPI_InitTypeDef			SPI_InitStruct;//过会儿要玩硬件SPI所以把这个结构体弄好
	//下面开始配置GPIO
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);//要玩GPIOA所以开时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);//要玩GPIOB所以开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//要玩SPI1所以开时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	//初始化端口
	//A9背光推挽输出 A6屏幕RS脚也是推挽输出 A1SCL
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOA,&GPIO_InitStruct);//设定好后写入结构体
	//A开漏输入 A0 SDA
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//A2 A3 A4 A10输入接按键
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//打开上拉因为按键按下是对地导通
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//B1屏幕复位RST脚推挽输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOB,&GPIO_InitStruct);//设定好后写入结构体
	
	//SPI端口映射
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;//ck//da
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//端口配置成其他功能
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_1;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);//AF0功能就是接到硬件SPI1上的
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);
	
	//硬件SPI配置
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;//单发送
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;//主机
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;//8位
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//频率
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;//高位先出
  SPI_InitStruct.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_Cmd(SPI1, ENABLE);//打开SPI1
}
//刷屏程序-----------------------------------------------------------------
void PInit(void)//屏幕初始化
{
	GPIOConfig();
	uint8_t i,j;
	RST_L;//屏幕硬件复位
	Delay(5000);
	RST_H;//屏幕硬件复位
	Delay(5000);
	SPIWCMD(0x11);
	SPIWCMD(0x11);
	SPIWCMD(0x11);//屏幕软件复位
	Delay(10000);
	for(i=0;i<96;i++)//开始刷初始化代码到屏幕
	{
		j=INIT[i];
		if(j==0xAA){i++;j=INIT[i];SPIWCMD(j);}//0xAA开头的则是CMD
		else SPIWDAT(j);
	}
	SPIWCMD(0x29);//打开显示
	while(SPI1->SR&0x80);//判断刷好才退出
}
void PBox(uint32_t POS)//划定显示方框
{
	Y1=POS;POS>>=8;
	X1=POS;POS>>=8;
	Y0=POS;POS>>=8;
	X0=POS;
	#ifdef OLDSCREEN
	SPIWCMD(0x2A);
	SPIWDAT(0x00);
	SPIWDAT(Y0+48);
	SPIWDAT(0x00);
	SPIWDAT(Y1+48);
	SPIWCMD(0x2B);
	SPIWDAT(0x00);
	SPIWDAT(X0+0);
	SPIWDAT(0x00);
	SPIWDAT(X1+0);
	SPIWCMD(0x2C);
	while(SPI1->SR&0x80);
	#endif
	#ifdef NEWSCREEN
	SPIWCMD(0x2A);
	SPIWDAT(0x00);
	SPIWDAT(Y0+67);
	SPIWDAT(0x00);
	SPIWDAT(Y1+67);
	SPIWCMD(0x2B);
	SPIWDAT(0x00);
	SPIWDAT(X0+2);
	SPIWDAT(0x00);
	SPIWDAT(X1+2);
	SPIWCMD(0x2C);
	while(SPI1->SR&0x80);
	#endif
}
void PSquare(uint32_t POS)//画方框填充CB色
{
	uint32_t c;
	PBox(POS);
	c=(X1-X0+1)*(Y1-Y0+1);
	while(c)
	{
		SPIW2DAT(CB);
		c--;
	}
}
void PLogo(void)//显示256色的开机画面
{
	uint16_t i;
	PBox(0x00007F3F);//设定窗口，上下有黑边
	for(i=0;i<8192;i++)
	{
		SPIW2DAT(C256[Logo[i]]);//刷Logo数组并从256色转成65536色
	}
}
void PString(char *p,uint16_t POS)
{
	uint8_t y;
	uint32_t P;
	uint32_t i,j,k;
	y=POS;P=POS<<16;
	P=P+0x7F07+y;
	PBox(P);
	while(*p!='\0')
	{
		i=*p*5-160;
		for(j=0;j<5;j++)
		{
			y=ASCS[i+j];
			for(k=0;k<8;k++)
			{
				if(y&0x01)SPIW2DAT(CF);else SPIW2DAT(CB);
				y>>=1;
			}
		}
		SPIWx8(CB);
		p++;
		if(*p=='\n'){p++;P+=0x00080008;PBox(P);}
	}
}
void PNum(uint32_t Num,uint16_t POS)
{
	char dat[6];
	dat[5]='\0';
	dat[4]=Num%10+48;Num/=10;
	dat[3]=Num%10+48;Num/=10;
	dat[2]=Num%10+48;Num/=10;
	dat[1]=Num%10+48;Num/=10;
	dat[0]=Num%10+48;
	PString(dat,POS);
}
void PHex(uint8_t Num,uint16_t POS)
{
	char dat[3];
	dat[2]='\0';
	dat[1]=Num%16+48;Num/=16;
	dat[0]=Num%16+48;
	if(dat[1]>57)dat[1]+=7;
	if(dat[0]>57)dat[0]+=7;
	PString(dat,POS);
}
void ERR(char *p)
{
	CF=RED;CB=BLA;
	PString(p,0x0000);
	while(1);
}
//硬件SPI----------------------------------------------------------------------------------
void SPIWDAT(uint8_t dat)//写一个字节数据
{
	while(SPI1->SR&0x80);//等待上一个数据传完
	RS_H;//再设置RS脚状态
	*(uint8_t *)(&SPI1->DR) = dat;//最后写入新的数据
}

void SPIW2DAT(uint16_t dat)//写2个字节数据
{
	while(SPI1->SR&0x80);
	RS_H;
	SPI1->DR = dat;
}

void SPIWCMD(uint8_t dat)//写1个字节指令
{
	while(SPI1->SR&0x80);
	RS_L;
	*(uint8_t *)(&SPI1->DR) = dat;
}

void SPIWx8(uint16_t dat)//发8次数据
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		SPIW2DAT(dat);
	}
}

//简易延迟----------------------------------------------------------------------------------
void Delay(__IO uint32_t i)//每次循环i-1直到i=0退出
{
	while(i--);
}
