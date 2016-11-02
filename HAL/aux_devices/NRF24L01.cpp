#include "NRF24L01.h"

#include <HAL/Interface/ISysTimer.h>

using namespace HAL;

namespace devices
{

#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define NRF_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前

#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
	//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
	//bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
	//bit5:数据发送完成中断;bit6:接收数据中断;

#define TX_ADR_WIDTH    5   //5字节的地址宽度
#define RX_ADR_WIDTH    5   //5字节的地址宽度
//uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb0,0x3d,0x12,0x34,0x01}; //发送地址
//uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0xb0,0x3d,0x12,0x34,0x01}; //发送地址
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xE7,0xE7,0xE7,0xE7,0xE7}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xC2,0xC2,0xC2,0xC2,0xC2}; //发送地址
#define TX_PLOAD_WIDTH  32
#define RX_PLOAD_WIDTH  32



#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define FIFO_STATUS     0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
	//bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
int NRF24L01::init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *ce)
{
	this->spi = spi;
	this->cs = cs;
	this->ce = ce;

	cs->set_mode(MODE_OUT_PushPull);
	ce->set_mode(MODE_OUT_PushPull);

	cs->write(true);
	ce->write(false);

	spi->set_speed(10000000);
	spi->set_mode(0, 0);

	systimer->delayms(100);
	power_on();

	write_cmd(FLUSH_TX, NOP);
	write_cmd(FLUSH_RX, NOP);

	uint8_t buf[5] = {0xc2, 0xc2, 0xc2, 0xc2, 0xc2, };
	write_cmd(TX_ADDR | NRF_WRITE_REG, buf, 5);
	for(int i=0; i<5; i++)
		buf[i] = 0xff;
	read_cmd(TX_ADDR, buf, 5);

	for(int i=0; i<5; i++)
		if (buf[i] != 0xc2)
			return -1;

	write_cmd(TX_ADDR | NRF_WRITE_REG, TX_ADDRESS, TX_ADR_WIDTH);
	write_cmd(RX_ADDR_P0 | NRF_WRITE_REG, RX_ADDRESS, RX_ADR_WIDTH);
	write_reg(EN_AA, 0);
	write_reg(EN_RXADDR, 3);
	write_reg(SETUP_RETR, 0);
	write_reg(RF_CH, 24);
	write_reg(RX_PW_P0, RX_PLOAD_WIDTH);
	write_reg(RX_PW_P1, RX_PLOAD_WIDTH);
	write_reg(RF_SETUP, 0x27);
	write_reg(CONFIG, 0x0e);
		
	write_reg(7, read_reg(7));	// clear all interrupt

	rf_on(false);

	return 0;
}

int NRF24L01::rf_on(bool rx)		// power up and enter standby-I, should be called only in power down state
{
	write_reg(CONFIG, (read_reg(CONFIG) & 0xfe) | (rx ? 1 : 0));

	ce->write(true);
	return 0;
}

int NRF24L01::rf_off()
{
	ce->write(false);
}

int NRF24L01::write_tx(const uint8_t *data, int count)
{
	write_reg(STATUS, read_reg(STATUS));
	write_cmd(WR_TX_PLOAD, data, count);

	return 0;
}

int NRF24L01::read_rx(uint8_t *data, int maxcount)
{
	if (maxcount > 32)
		maxcount = 32;

	read_cmd(RD_RX_PLOAD, data, maxcount);

	return maxcount;
}

int NRF24L01::power_on()		// power up and enter standby-I, should be called only in power down state
{
	write_reg(CONFIG, read_reg(CONFIG) | 2);

	return 0;
}
int NRF24L01::power_off()	// power down the whole chip.
{
	write_reg(CONFIG, read_reg(CONFIG) & (~2));
	return 0;
}


int NRF24L01::write_cmd(uint8_t cmd, const uint8_t *data, int count)
{
	cs->write(false);
	spi->txrx(cmd);
	for(int i=0; i<count; i++)
		spi->txrx(data[i]);
	cs->write(true);

	return 0;
}

int NRF24L01::read_cmd(uint8_t cmd, uint8_t *data, int count)
{
	cs->write(false);
	spi->txrx(cmd);
	for(int i=0; i<count; i++)
		data[i] = spi->txrx(0);
	cs->write(true);

	return 0;
}

uint8_t NRF24L01::read_cmd(uint8_t cmd)
{
	uint8_t v;
	read_cmd(cmd, &v, 1);

	return 1;
}
void NRF24L01::write_cmd(uint8_t cmd, uint8_t data)
{
	write_cmd(cmd, &data, 1);
}

uint8_t NRF24L01::read_reg(uint8_t cmd)
{
	uint8_t v;
	read_cmd(cmd & 0x1f, &v, 1);

	return v;
}
void NRF24L01::write_reg(uint8_t cmd, uint8_t data)
{
	write_cmd((cmd & 0x1f) | NRF_WRITE_REG, &data, 1);
}

}