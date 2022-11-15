#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4HSBulk.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4ADC.h>
#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include <Protocol/common.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_dac.h>

extern "C"
{
#include "A5133/myRadio.h"
#include "A5133/A5133_hal.h"
}

using namespace STM32F4;
using namespace HAL;

F4GPIO led0(GPIOB, GPIO_Pin_15);


extern F4Interrupt trig;

extern F4GPIO pa_en;
extern F4GPIO lna_en;
extern F4GPIO ctr;
extern F4GPIO csA;

extern F4SPI spi;

uint8_t regs[64];

void A5133_write(uint8_t add, uint8_t data)
{
	csA.write(0);
	systimer->delayus(1);		
	spi.txrx(add);
	systimer->delayus(1);	
	spi.txrx(data);
	csA.write(1);
}

static rfRxPacket_ts rfRecvPacket;

bool tx_busy = false;
bool restart_rx = false;
bool extract_payload = false;

int64_t last_valid_pkt = 0;
float dt_lpf = 10000;
extern "C" void RF_EXT_PA_TO_IDLE();

bool tx = true;

void next()
{
	if (tx)
	{

	}
	else
	{
		if (restart_rx)
		{
			restart_rx = false;
			myRadio_receiver();
		}
		
		if (extract_payload)
		{
			RF_FIFORead(rfRecvPacket.payload, 64);
			
			if (memcmp(rfRecvPacket.payload, "PING", strlen("PING")) == 0)
			{
				led0.write(1);
				systimer->delayus(1);
				led0.write(0);
				
				int64_t ts = systimer->gettime();
				int dt = ts - last_valid_pkt;
				last_valid_pkt = ts;
				
				if (dt < 1000000)
					dt_lpf = dt * 0.002f + dt_lpf * 0.998f;
				else
					dt_lpf = 1000000;
			}			
		}
	}
}


void rfRx_callback(uint8_t status, rfRxPacket_ts packet)
{    
    
    switch (status)
    {
        case RX_STA_SECCESS:
        {
            //event_post(EVENT_RF_GET_RX_PACKET);
			rfRecvPacket = packet;
			restart_rx = true;
			extract_payload = true;			

        }
        break;
		
        case RX_STA_TIMEOUT:
        {
            //event_post(EVENT_RF_RX_ERROR);
			restart_rx = true;
        }
        break;
		
        case RX_STA_PAYLOAD_ERROR:
        {
            //event_post(EVENT_RF_RX_ERROR);
			restart_rx = true;
        }
        break;
		
        case TX_STA_SECCESS:
        {
			tx_busy = false;
			RF_EXT_PA_TO_IDLE();
			//myRadio_abort();
			led0.write(1);
			systimer->delayus(5);
			led0.write(0);
            //event_post(EVENT_RF_PACKET_RX);
        }
        break;
		
        default:
            break;
    }
	
	
	next();

}


int main()
{
	systimer->delayms(500);
	led0.set_mode(HAL::MODE_OUT_PushPull);
	led0.write(0);
	
	myRadio_init(0, rfRx_callback);
	myRadio_setFrequency(0);
	myRadio_setTxPower(0);
	
	
	F4GPIO scl(GPIOA, GPIO_Pin_10);
	F4GPIO sda(GPIOA, GPIO_Pin_9);
	I2C_SW i2c(&scl, &sda);
	i2c.set_speed(1);
	devices::OLED96 oled;
	oled.init(&i2c, 0x78);
	oled.show_str(0, 0, "HelloWorld");	
	oled.clear();
	
	/*
	myRadio_setCtrl(RADIO_EXT_CONTROL_TX_UNMODULATED, 0);
	while(1)
		;
	*/
	
	if (tx)
	{
		oled.clear();
		oled.show_str(0, 0, "TX start");
		
		int64_t ltx = systimer->gettime();

		while(1)
		{
			//myRadio_process();
			
			int64_t ts = systimer->gettime();

			if (ts > ltx + 400)
			//if (!tx_busy)
			{
				//led0.write(1);

				rfTxPacket_ts rfTxPacket;
				rfTxPacket.len = 64;
				memcpy(rfTxPacket.payload, "PING", rfTxPacket.len);
				myRadio_transmit(&rfTxPacket);
				
				tx_busy = true;				
				
				ltx = ts;
			}
		}
	}
	
	else
	{
		oled.clear();
		oled.show_str(0, 0, "RX start");

		myRadio_receiver();
		while(1)
		{


			if (systimer->gettime() > last_valid_pkt + 1000000)
			{
				oled.show_str(0, 0, "LOS        ");				
			}
			else
			{
				char tmp[100];
				sprintf(tmp, "%.0f    ", 1000000/dt_lpf);
				oled.show_str(0, 0, tmp);				
				sprintf(tmp, "%d dbm    ", rfRecvPacket.rssi);
				oled.show_str(0, 1, tmp);				
			}
		}
	}
}


int main2()
{
	led0.set_mode(HAL::MODE_OUT_PushPull);
	led0.write(1);
	
	csA.write(0);
	spi.txrx(0x90);
	csA.write(1);
		systimer->delayus(10000);
		
	A5133_write(0x0B, (0x00 << 2) | 0);
	A5133_write(0x0C, (0x00 << 2) | 0);
	
	for(int i=0x09; i<0x40; i++)
	{
		csA.write(0);
		systimer->delayus(1);		
		spi.txrx(i | 0x40);
		systimer->delayus(1);	
		regs[i] = spi.txrx(0);
		csA.write(1);
		
		
		//systimer->delayus(1000);
		
	}

	

	while(1)
	{
		uint8_t add = 0x0e;
		
		
		/*

		csA.write(0);
		systimer->delayus(1);	
		spi.txrx(add);
		systimer->delayus(1);	
		spi.txrx(0x55);
		csA.write(1);
		
		
		
		systimer->delayus(20);
		*/
				
		
		
		
		csA.write(0);
		systimer->delayus(1);		
		spi.txrx(add | 0x40);
		systimer->delayus(1);	
		regs[0] = spi.txrx(0);
		systimer->delayus(1);	
		csA.write(1);
		
	}
}

