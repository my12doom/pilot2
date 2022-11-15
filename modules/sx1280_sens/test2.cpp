extern "C" 
{
#include "libsx128x/lib/sx1280.h"
}

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>

using namespace STM32F4;
void rf_off();

bool tx_done = false;
bool rx_done = false;
void OnTxDone( SX1280_t *sx1280 )
{
	tx_done = true;
	rf_off();
}

void OnRxDone( SX1280_t *sx1280 )
{
	rx_done = true;
}

int64_t tx_start;
int tx_timeout_dt = 0;
void OnTxTimeout( SX1280_t *sx1280 )
{
	tx_timeout_dt = systimer->gettime() - tx_start;
	rf_off();
}

void OnRxTimeout( SX1280_t *sx1280 )
{
	//printf("RX Timeout");
	rf_off();
}

void OnRxError( SX1280_t *sx1280, IrqErrorCode_t errorCode )
{
}

void OnRangingDone( SX1280_t *sx1280, IrqRangingCode_t val )
{
}

void OnCadDone( SX1280_t *sx1280, bool channelActivityDetected )
{
}

RadioCallbacks_t Callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    NULL,             // rangingDone
    NULL,             // cadDone
};

#define RADIO_CTX_PIN       GPIO_Pin_1
#define RADIO_CTX_PORT      GPIOB

#define RADIO_CPS_PIN       GPIO_Pin_12
#define RADIO_CPS_PORT      GPIOA

#define RADIO_CSD_PIN       GPIO_Pin_0
#define RADIO_CSD_PORT      GPIOA

#define ANT_SEL_PIN         GPIO_Pin_0
#define ANT_SEL_PORT        GPIOB

F4GPIO ctx(RADIO_CTX_PORT, RADIO_CTX_PIN);
F4GPIO cps(RADIO_CPS_PORT, RADIO_CPS_PIN);
F4GPIO csd(RADIO_CSD_PORT, RADIO_CSD_PIN);
F4GPIO ant(ANT_SEL_PORT, ANT_SEL_PIN);
F4GPIO led(GPIOC, GPIO_Pin_13);

void ant_sel(int a)
{
	ant.write(a);
}

void rf_tx()
{
	cps.write(0);
	csd.write(1);
	ctx.write(1);
}

void rf_rx()
{
	cps.write(1);
	csd.write(1);
	ctx.write(0);
}

void rf_bypass()
{
	cps.write(0);
	csd.write(1);
	ctx.write(0);
}

void rf_off()
{
	cps.write(0);
	csd.write(0);
	ctx.write(0);	
}

uint8_t BUFFER_SIZE = 32;
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US
#define RX_TIMEOUT_VALUE                            100 // ms

#define TX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US
#define TX_TIMEOUT_VALUE                            100 // ms

uint8_t rssi = 0;
float rssif;
bool tx = 0;
bool rx = 1;
RadioStatus_t radio_status;

bool heat_run = false;
void btn_cb(void *p, int flags)
{
	heat_run = !heat_run;
}

int HEAT()
{
	F4Interrupt btn;
	btn.init(GPIOB, GPIO_Pin_15, HAL::interrupt_rising);
	btn.set_callback(btn_cb, NULL);
	F4GPIO test_on(GPIOA, GPIO_Pin_8);
	F4GPIO pwm1(GPIOB, GPIO_Pin_0);
	test_on.set_mode(HAL::MODE_OUT_PushPull);
	pwm1.set_mode(HAL::MODE_OUT_PushPull);

	while(1)
	{
		test_on.write(heat_run);
		
		if (heat_run)
			pwm1.write((systimer->gettime() % 100000) < 1000);
		else
			pwm1.write(0);
	}
}

int main()
{
	//HEAT();
	cps.set_mode(HAL::MODE_OUT_PushPull);
	csd.set_mode(HAL::MODE_OUT_PushPull);
	ctx.set_mode(HAL::MODE_OUT_PushPull);
	ant.set_mode(HAL::MODE_OUT_PushPull);
	led.set_mode(HAL::MODE_OUT_PushPull);
	ant.write(1);
	ant_sel(1);
	rf_off();
	led.write(0);
	

	
    ModulationParams_t modulationParams;
	PacketParams_t packetParams;
	
    //printf( "\nPing Pong running in LORA mode\n\r" );
    modulationParams.PacketType = PACKET_TYPE_FLRC;
    modulationParams.Params.Flrc.BitrateBandwidth = FLRC_BR_0_325_BW_0_3;
    modulationParams.Params.Flrc.CodingRate = FLRC_CR_3_4;
    modulationParams.Params.Flrc.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;
	
	
    packetParams.PacketType = PACKET_TYPE_FLRC;
    packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    packetParams.Params.Flrc.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Flrc.HeaderType = RADIO_PACKET_VARIABLE_LENGTH;
    packetParams.Params.Flrc.PayloadLength = BUFFER_SIZE;
    packetParams.Params.Flrc.CrcLength = RADIO_CRC_3_BYTES;
    packetParams.Params.Flrc.Whitening = RADIO_WHITENING_OFF;
	
	/*
	modulationParams.PacketType = PACKET_TYPE_GFSK;
    modulationParams.Params.Gfsk.BitrateBandwidth = GFSK_BLE_BR_0_125_BW_0_3;
    modulationParams.Params.Gfsk.ModulationIndex = GFSK_BLE_MOD_IND_1_00;
    modulationParams.Params.Gfsk.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    packetParams.PacketType = PACKET_TYPE_GFSK;
    packetParams.Params.Gfsk.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Gfsk.SyncWordLength = GFSK_SYNCWORD_LENGTH_5_BYTE;
    packetParams.Params.Gfsk.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Gfsk.HeaderType = RADIO_PACKET_VARIABLE_LENGTH;
    packetParams.Params.Gfsk.PayloadLength = BUFFER_SIZE;
    packetParams.Params.Gfsk.CrcLength = RADIO_CRC_3_BYTES;
    packetParams.Params.Gfsk.Whitening = RADIO_WHITENING_ON;
	
	
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
    modulationParams.Params.LoRa.Bandwidth = LORA_BW_1600;
    modulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;

    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = 12;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = BUFFER_SIZE;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	*/
	
	printf("\n\n\n\n");
	SX1280_t sx1280;
	
	SX1280Init(&sx1280, &Callbacks);

    SX1280SetRegulatorMode(&sx1280,  USE_LDO ); // Can also be set in LDO mode but consume more power
    SX1280SetStandby(&sx1280,  STDBY_RC );
    SX1280SetPacketType(&sx1280,  modulationParams.PacketType );
    SX1280SetModulationParams(&sx1280,  &modulationParams );
    SX1280SetPacketParams(&sx1280,  &packetParams );
    SX1280SetRfFrequency(&sx1280,  2484000000 );
    SX1280SetBufferBaseAddresses(&sx1280,  0x00, 0x00 );
    SX1280SetTxParams(&sx1280,  13, RADIO_RAMP_02_US );
		
    SX1280SetPollingMode( &sx1280);
	//SX1280SetInterruptMode(&sx1280);
    SX1280SetDioIrqParams(&sx1280,  RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    SX1280SetRx(&sx1280,  ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
	
	uint8_t Buffer[BUFFER_SIZE];
	strcpy((char*)Buffer, "HelloWorld");
	for(int i=0; i<32; i++)
		Buffer[i] = '0' + i;
	
	int64_t last_rx_time = systimer->gettime();
	int64_t last_tx_time = systimer->gettime();
	while(1)
	{
		SX1280ProcessIrqs(&sx1280);
		
		if (rx)
		{
			//rssi = SX1280GetRssiInst(&sx1280);
			rssif = -rssi/2.0f;
			rf_rx();
			if (systimer->gettime() > last_rx_time + 1200000)
			{
				printf("RX START\n");
				SX1280GetIrqStatus(&sx1280);
				SX1280ProcessIrqs(&sx1280);
				SX1280SetDioIrqParams(&sx1280,  0xffff, 0xffff, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				SX1280SetRx(&sx1280,  ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
				last_rx_time = systimer->gettime();
			}
		}
		else if (tx)
		{
			if (systimer->gettime() > last_tx_time + 4000 || tx_done)
			{
				ant.toggle();
				ant.write(1);
				//SX1280SetStandby( STDBY_RC );
				SX1280SetDioIrqParams(&sx1280,  TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				SX1280SendPayload(&sx1280,  Buffer, BUFFER_SIZE, ( TickTime_t ){ TX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
				radio_status = SX1280GetStatus(&sx1280);
				tx_start = systimer->gettime();
				last_tx_time = systimer->gettime();
				rf_tx();
				led.toggle();
			}
		}
		else
		{
			rf_off();
		}

	}
	
	while(1)
	{
		SX1280SetDioIrqParams(&sx1280,  TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
		SX1280SendPayload(&sx1280,  Buffer, BUFFER_SIZE, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
		SX1280ProcessIrqs(&sx1280);
	}
}
