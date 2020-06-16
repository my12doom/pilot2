#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4UART2.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4VCP.h>
#include <HAL/Interface/II2c.h>
#include <modules/utils/param.h>
#include <HAL/aux_devices/LoraRFHelper.h>
#include <HAL/aux_devices/Sx127x.h>

#include "ZLink.h"
#include "ZLinkWireless.h"
#include "ZLinkManagementPacket.h"

extern "C"
{
#include <modules/utils/SEGGER_RTT.h>
}

using namespace HAL;
using namespace STM32F4;

#define LOGE(...)

// configuration
#define RTK_PRIORITY 0
#define QUEUE_COUNT 10
#define RTK_MTU 50
#define RTK_interval 50000
#define TX_INTERVAL_LORA 20000		// time for ack packet airtime and node processing time
#define TX_INTERVAL_GFSK 3000		// time for ack packet airtime and node processing time
#define MAX_TX_QUEUE_TIME 2500
param lora_mode("lora", 1);
param uplink_freq("fup", 429);
param downlink_freq("fdwn", 437);

// hardware/board
static STM32F4::F4GPIO led_lora1(GPIOB, GPIO_Pin_1);
static STM32F4::F4GPIO led_lora2(GPIOB, GPIO_Pin_4);
static STM32F4::F4GPIO led_usart_rtk(GPIOB, GPIO_Pin_0);
static STM32F4::F4GPIO led_usb(GPIOB, GPIO_Pin_5);
F4GPIO rst(GPIOC, GPIO_Pin_14);
static F4UART2 uart_rtk(USART1);
F4Interrupt lora1_DIO0;
F4Interrupt lora1_DIO1;
F4Interrupt lora2_DIO0;
F4Interrupt lora2_DIO1;
F4Timer t5(TIM5);

extern "C" void TIM5_IRQHandler(void)
{
	t5.call_callback();
}

F4Timer t3(TIM3);

extern "C" void TIM3_IRQHandler(void)
{
	t3.call_callback();
}

// TX with ack queue
sx127x_packet flying_packets[QUEUE_COUNT];		// un-ACKed packets
uint16_t flying_packets_usb_sequence_id[QUEUE_COUNT];
int flying_packet_count = 0;

// internal states
int sequence_id = 0;
static int64_t ref_dt = 0;
static uint8_t aes_key[32] = {0x85, 0xA3};

// node states
ZLinkNodeState node_list[512] = {0};
int node_count = 0;
bool probe_nodes = false;
int64_t next_probing = 0;
int probing_interval = 2000000;
int ack_list[25];
uint16_t ack_list_nodeid[25];
int ack_index = 0;

// RTK
int64_t next_rtk_frame = 0;

// helpers
static SX127xManager mtx;
static SX127xManager mrx;
static sx127x_packet p;

int usb_ack(IUART *usb, int sequence_id, bool end, bool ack = true)
{
	static ZLinkUSBFrame *out = new_frame(MAX_USB_DATA_SIZE, 0, 0, 0, NULL);
	if (!out)
		return -1;

	out->cmd = ack ? cmd_nack : cmd_ack;
	out->size = 0;
	out->sequence_id = sequence_id;
	out->flag = end ? frame_flag_END : 0;
	update_crc(out);

	return usb->write(out, out->size + HEADER_SIZE);
}

int usb_out(IUART *usb, uint8_t cmd, uint16_t sequence_id, uint8_t flag, const uint8_t *data, int data_size)
{
	ZLinkUSBFrame *out = new_frame(MAX_USB_DATA_SIZE, 0, 0, 0, NULL);
	if (!out)
		return -1;

	memcpy(out->data, data, data_size);
	out->cmd = cmd;
	out->size = data_size;
	out->sequence_id = sequence_id;
	out->flag = flag;
	update_crc(out);

	free_frame(out);

	return usb->write(out, out->size + HEADER_SIZE);
}

static int64_t get_ref_time()
{
	return systimer->gettime() + ref_dt;
}


int update_rssi(int16_t nodeid, int rssi)
{
	for(int i=0; i<node_count; i++)
	{
		if (node_list[i].nodeid == nodeid)
			node_list[i].rssi = rssi;
	}

	return 0;
}

int reset_sx127x()
{
	rst.write(0);
	systimer->delayms(1);
	rst.write(1);
	mtx.set_lora_mode(lora_mode);
	mrx.set_lora_mode(lora_mode);
	mtx.set_tx_interval(lora_mode > 0.5f ? TX_INTERVAL_LORA : TX_INTERVAL_GFSK);
	mrx.set_tx_interval(lora_mode > 0.5f ? TX_INTERVAL_LORA : TX_INTERVAL_GFSK);
	
	// magic
	p.size = 20;
	mtx.write(p, 0);
	systimer->delayms(25);
	mrx.write(p, 0);
	systimer->delayms(25);
	
	return 0;
}

static int imax(int a, int b)
{
	if(a>b)
		return a;
	return b;
}

extern "C" void HardFault_Handler(void)
{
  // RESET!
  SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      | 
                 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) | 
                 SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
  __DSB();                                                     /* Ensure completion of memory access */              

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

	int tx = 0;
	int rx = 0;
	int badrx = 0;

int handle_air_rx(F4VCP &usb, ZLinkUSBFrame *usb_out)
{	
	int manager_result = -1;
	do
	{
		bool istx = true;
		manager_result = mtx.read(&p);
		if (manager_result < 0)
		{
			manager_result = mrx.read(&p);
			istx = false;
		}
		if (manager_result < 0)
			continue;
				
		led_lora2.toggle();
		WirelessPacket * wp = (WirelessPacket*) p.data;
		if (istx)
			update_rssi(wp->src_node_id, p.power);
		if (!check_crc(wp) && (wp->dest_node_id == NODE_STATION || wp->dest_node_id == NODE_BROADCAST))
		{
			LOGE("invalid RX(CRC error), id=%d, rssi=%d\n", wp->sequence_id, p.power);
			badrx++;
		}
		else
		{
			// ACK if flaged
			if (wp->type & packet_flag_need_ack && wp->dest_node_id == NODE_STATION)
			{
				sx127x_packet tx;
				tx.size = PACKET_HEADER_SIZE;

				WirelessPacket * ptx = (WirelessPacket *)tx.data;
				ptx->type = packet_ack;
				ptx->length = 0;
				ptx->src_node_id = NODE_STATION;
				ptx->dest_node_id = wp->src_node_id;
				ptx->sequence_id = wp->sequence_id;

				update_crc(ptx);

				mtx.write(tx, 0);
				mtx.flush();

				// discard duplicated(ack already sent) packets
				bool discard = false;
				for(int i=0; i<sizeof(ack_list)/sizeof(ack_list[0]); i++)
				{
					if (ack_list[i] == wp->sequence_id && ack_list_nodeid[i] == wp->src_node_id)
					{
						discard = true;
						break;
					}
				}

				if (discard)
					continue;

				// add to ack list
				ack_list[ack_index] = wp->sequence_id;
				ack_list_nodeid[ack_index] = wp->src_node_id;
				ack_index = (ack_index+1) % sizeof(ack_list)/sizeof(ack_list[0]);
			}
			
			// remove ack flag
			wp->type &= 0x7f;

			// ack packets of uplink
			if (wp->type == packet_ack)
			{
				bool ack_found = false;
				for(int i=0; i<flying_packet_count ;i++)
				{
					WirelessPacket * fwp = (WirelessPacket*) flying_packets[i].data;
					if (fwp->sequence_id == wp->sequence_id)
					{
						// ACK done
						//RTT_printf("air ACK:%d, retry left:%d, t=%lld\n", wp->sequence_id, flying_packets[i].retries_left, systimer->gettime());
						ack_found = true;

						// USB result
						usb_out->cmd = cmd_tx_withack;
						usb_out->size = 1;
						usb_out->sequence_id = flying_packets_usb_sequence_id[i];
						usb_out->flag = frame_flag_END;
						usb_out->data[0] = 1;
						update_crc(usb_out);

						usb.write(usb_out, usb_out->size + HEADER_SIZE);

						// remove from queue
						for(int j=i; j<flying_packet_count-1; j++)
						{
							flying_packets[j] = flying_packets[j+1];
							flying_packets_usb_sequence_id[j] = flying_packets_usb_sequence_id[j+1];
						}

						flying_packet_count--;

					}
				}

				if (!ack_found)
				{
					WirelessPacket * fwp = (WirelessPacket*) flying_packets[0].data;
					LOGE("unexpected ACK %d/%d?, ts=%lld\n", fwp->sequence_id, wp->sequence_id, systimer->gettime());
				}
			}

			// downlink payload
			if ( (wp->type == packet_payload || wp->type == packet_result)
				 && wp->dest_node_id == NODE_STATION)
			{
				usb_out->cmd = cmd_rx;
				usb_out->size = p.size;
				usb_out->sequence_id = SEQUENCE_DOWNLINK;
				usb_out->flag = frame_flag_END;
				memcpy(usb_out->data, p.data, p.size);
				update_crc(usb_out);

				usb.write(usb_out, usb_out->size + HEADER_SIZE);
			}

			// probe response
			if (wp->type == packet_probe_respond)
			{
				int nodeid = wp->src_node_id;
				int found = -1;
				ZLinkNodeState *state = wp->length >= sizeof(ZLinkNodeState) ? (ZLinkNodeState *)wp->payload : NULL;
				for(int i=0; i<node_count; i++)
				{
					if (node_list[i].nodeid == nodeid)
					{
						found = i;
						if (istx)
							node_list[i].rssi = p.power;
						if (state)
						{
							node_list[i].rssi_onboard = state->rssi_onboard;
							node_list[i].last_update = get_ref_time();
						}
						break;
					}
				}
				if (found == -1)
				{
					if (state)
						node_list[node_count].rssi_onboard = state->rssi_onboard;
					node_list[node_count].rssi = p.power;
					node_list[node_count++].nodeid = nodeid;
				}
			}
			
			rx++;

		}
		
		LOGE("RX:%dbyte,%ddbm, TX/RX=%d/%d pkts, id:%d, lost = %d-%d\n", p.size, p.power, tx, rx, ((WirelessPacket*)p.data)->sequence_id, tx-rx, badrx);
	}  while(manager_result == 0);
	
	return 0;
}

static void to_dfu(void){
    do {SYSCFG->MEMRMP &= ~(SYSCFG_MEMRMP_MEM_MODE);
	 SYSCFG->MEMRMP |= SYSCFG_MEMRMP_MEM_MODE_0;
	}while(0);

    // arm-none-eabi-gcc 4.9.0 does not correctly inline this
    //     //     // MSP function, so we write it out explicitly here.
    __set_MSP(*((uint32_t*) 0x00000000));
    //__ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

    ((void (*)(void)) *((uint32_t*) 0x00000004))();

    while (1);
}


int main()
{
	//to_dfu();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	
	F4VCP usb;
	PWR_BackupAccessCmd(ENABLE);

	// init SX1278
	rst.set_mode(MODE_OUT_PushPull);
	rst.write(0);
	systimer->delayms(1);
	rst.write(1);

	// LORA1(TX) init
	F4GPIO nss1(GPIOA, GPIO_Pin_4);
	F4SPI spi1(SPI1);
	lora1_DIO0.init(GPIOA, GPIO_Pin_8, interrupt_rising);
	lora1_DIO1.init(GPIOA, GPIO_Pin_15, interrupt_rising_or_falling);

	led_lora1.set_mode(MODE_OUT_OpenDrain);
	nss1.set_mode(MODE_OUT_PushPull);

	SX127x x_t;
	if (x_t.init(&spi1, &nss1, NULL, NULL)<0)
	{
		while(1)
		{
			led_lora1.toggle();
			systimer->delayms(500);
		}
	}
	x_t.set_frequency(uplink_freq);
	x_t.set_lora_mode(lora_mode);
	
	mtx.init(&x_t, &lora1_DIO0, &t5, &lora1_DIO1);
	mtx.set_tx_interval((lora_mode > 0.5f) ? TX_INTERVAL_LORA : TX_INTERVAL_GFSK);
	mtx.set_frequency(uplink_freq, downlink_freq);
	mtx.set_aes(aes_key, 32);

	// LORA2(RX) init
	F4GPIO nss2(GPIOB, GPIO_Pin_12);
	F4SPI spi2(SPI2);
	lora2_DIO0.init(GPIOB, GPIO_Pin_10, interrupt_rising);
	lora2_DIO1.init(GPIOB, GPIO_Pin_9, interrupt_rising_or_falling);

	SX127x x_r;
	if (x_r.init(&spi2, &nss2, NULL, NULL)<0)
	{
		while(1)
		{
			led_lora2.toggle();
			systimer->delayms(500);
		}
	}

	x_r.set_lora_mode(lora_mode);
	mrx.init(&x_r, &lora2_DIO0, &t3, &lora2_DIO1);
	mtx.set_tx_interval(lora_mode > 0.5f ? TX_INTERVAL_LORA : TX_INTERVAL_GFSK);
	mrx.set_frequency(uplink_freq, downlink_freq);
	mrx.set_aes(aes_key, 32);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_Init(&NVIC_InitStructure);

	// magic
	p.size = 10;
	mtx.write(p, 0);
	systimer->delayms(25);	
	mrx.write(p, 0);
	systimer->delayms(25);	

	ZLinkUSBFrame *in = new_frame(MAX_USB_DATA_SIZE, 0, 0, 0, NULL);
	if (!in)
		return -1;

	ZLinkUSBFrame *out = new_frame(MAX_USB_DATA_SIZE, 0, 0, 0, NULL);
	if (!out)
	{
		free_frame(in);
		return -1;
	}

	ZLinkUSBFrame *result = new_frame(MAX_USB_DATA_SIZE, 0, 0, 0, NULL);
	if (!result)
	{
		free_frame(in);
		free_frame(out);
		return -1;
	}

	// init things
	for(int i=0; i<sizeof(ack_list)/sizeof(ack_list[0]); i++)
		ack_list[i] = -1;
	
	uart_rtk.set_baudrate(57600);
	led_lora1.set_mode(MODE_OUT_OpenDrain);
	led_lora2.set_mode(MODE_OUT_OpenDrain);
	led_usart_rtk.set_mode(MODE_OUT_OpenDrain);
	led_usb.set_mode(MODE_OUT_OpenDrain);
	
	// sensitivity test
	/*
	mtx.set_frequency(437, 437);
	mrx.set_frequency(437, 437);
	mtx.set_lora_mode(true);
	mrx.set_lora_mode(true);
	while (1)
	{
#if 0
		p.size = 50;
		WirelessPacket *wp = (WirelessPacket*)p.data;
		wp->length = 40;
		update_crc(wp);
		mtx.write(p, 0);
		systimer->delayms(100);
#endif
		
		static sx127x_packet pr;
		WirelessPacket *wpr = (WirelessPacket*)pr.data;
		if (mrx.read(&pr) == 0)
		{
			led_lora2.toggle();
			printf("%dRX:%d\n", pr.power, check_crc(wpr) ? 1 : 0);
		}
		if (mtx.read(&pr) == 0)
		{
			led_lora2.toggle();
			printf("%dTX:%d\n", pr.power, check_crc(wpr) ? 1 : 0);
		}
	}
	*/
	
	// uart loopback test
	/*
	uart_rtk.set_baudrate(5000000);
	char tmp[400];
	uart_rtk.write(tmp, sizeof(tmp));
	while(1)
	{
		int c = uart_rtk.read(tmp, sizeof(tmp));
		
		if (c>0)
			uart_rtk.write(tmp, c);		
	}
	*/
	

	
	int64_t lastt = systimer->gettime();
	int64_t times[6] = {0};
	int64_t last_rgb = systimer->gettime();
	int rgb_id = 0;
	while(1)
	{
		// range test tool
		/*
		if (systimer->gettime() - last_rgb > 250000)
		{
			WirelessPacket * wp = (WirelessPacket*) p.data;
			wp->dest_node_id = NODE_BROADCAST;
			wp->src_node_id = NODE_STATION;
			wp->type = packet_management;
			wp->length = 4;
			wp->sequence_id = rgb_id;

			rgb_id ++;

			wp->payload[0] = management_RGB;
			wp->payload[1] = ((rgb_id+0) % 3) ? 255 : 0;
			wp->payload[2] = ((rgb_id+1) % 3) ? 255 : 0;
			wp->payload[3] = ((rgb_id+2) % 3) ? 255 : 0;

			update_crc(wp);

			p.size = sizeof(WirelessPacket) + 4;
			
			mtx.write(p, 0);
			led_lora1.toggle();

			last_rgb = systimer->gettime();
		}
		*/
		
		// performance counter
		int dt = systimer->gettime() - lastt;
		if (dt > 1000)
		{
			LOGE("performance warning:%d=", dt);
			for(int i=0; i<5; i++)
				LOGE("%d+", int(times[i+1]-times[i]));
			LOGE("\n");
		}
		lastt = systimer->gettime();
		
		bool txbusy = 0 == mtx.txqueue_space(1);

		// RX
		times[0] = systimer->gettime();
		handle_air_rx(usb, out);

		// USB frame handling
		times[1] = systimer->gettime();
		int c = search_frame(&usb, in);
		if (c>0)
		{
			led_usb.toggle();
			bool busy = false;
			bool more_data = false;
			bool end_of_sequence = true;
			result->size = 0;	// 0: no result
			WirelessPacket * wp = (WirelessPacket*) p.data;
			WirelessPacket * inwp = (WirelessPacket*) in->data;

			switch (in->cmd)
			{
				case cmd_tx_withack:
					busy = flying_packet_count >= QUEUE_COUNT;
					if (!busy)
					{
						inwp->type |= packet_flag_need_ack;
						inwp->sequence_id = sequence_id;
						sequence_id = (sequence_id+1) % SEQUENCE_DOWNLINK;
						update_crc((WirelessPacket*)in->data);
						p.size = inwp->length + PACKET_HEADER_SIZE;
						p.retries_left = 5;
						p.next_tx = 0;
						bool blocking = false;
						if (in->size >= inwp->length + PACKET_HEADER_SIZE + 2)
							blocking = inwp->payload[inwp->length+1];
						if (in->size >= inwp->length + PACKET_HEADER_SIZE + 1)
							p.retries_left = inwp->payload[inwp->length];
						
						if (blocking && (flying_packet_count > 0 /*|| !mtx.ready_for_next_tx()*/))
						{
							end_of_sequence = busy = true;
						}
						else
						{
							end_of_sequence = !blocking;
							memcpy(p.data, in->data, inwp->length + PACKET_HEADER_SIZE);
							LOGE("flying:%d, ts=%lld\n", sequence_id, systimer->gettime());
							
							flying_packets_usb_sequence_id[flying_packet_count] = in->sequence_id;
							flying_packets[flying_packet_count++] = p;
						}
					}
					break;

				case cmd_tx_oneshot:
					if (!txbusy)
					{
						wp->sequence_id = sequence_id;
						sequence_id = (sequence_id+1) % SEQUENCE_DOWNLINK;
					}
				case cmd_tx_raw:
					p.size = in->size;
					memcpy(p.data, in->data, in->size);

					if (!txbusy)
					{
						if (mtx.write(p, 1) < 0)
						{
							busy = true;
						}
						else
						{
							led_lora1.toggle();
							LOGE("sent:%d\n", ((WirelessPacket*)p.data)->sequence_id);
							tx ++;							
						}
					}
					else
					{
						busy = true;
					}
					break;

				case cmd_probing_start:
					if (in->size == 4)
						probing_interval = *(int32_t*)in->data;
					probe_nodes = true;
					next_probing = 0;	// start at once
					break;
				case cmd_probing_stop:
					probe_nodes = false;
					break;
				case cmd_network_set_aes:
					if(in->size == 32)
						memcpy(aes_key, in->data, in->size);
					mtx.set_aes(aes_key, 32);
					mrx.set_aes(aes_key, 32);
					break;

				case cmd_probing_get_list:
					// return a bitmap of available nodes
					result->size = 0;
					memset(result->data, 0, MAX_USB_DATA_SIZE);
					for(int i=0; i<node_count; i++)
					{
						int id = node_list[i].nodeid;
						int64_t timeout = get_ref_time() - 10000000;
						if(node_list[i].last_update > timeout && id < MAX_USB_DATA_SIZE*8)
						{							
							((uint8_t*)result->data)[id/8] |= 1<<(id%8);
							result->size = imax(result->size, id/8+1);
						}
					}
					result->cmd = cmd_probing_get_list;
					result->sequence_id = in->sequence_id;
					result->flag = frame_flag_END;
					end_of_sequence = false;
					more_data = true;
					break;
				case cmd_probing_get_state:
					// return the actual state
					if (in->size >= 4)
					{
						result->size = 0;
						int nodeid = *(int*)in->data;
						for(int i=0; i<node_count; i++)
						{
							if (node_list[i].nodeid == nodeid)
							{
								result->cmd = cmd_probing_get_state;
								result->sequence_id = in->sequence_id;
								result->flag = frame_flag_END;
								result->size = sizeof(ZLinkNodeState);
								memcpy(result->data, &node_list[i], sizeof(ZLinkNodeState));
								end_of_sequence = false;
								more_data = true;
								break;
							}
						}
					}
					else
					{
						busy = true;
					}
					break;

				case cmd_network_set_time:
					if(in->size == 8)
						ref_dt = *(int64_t*)in->data - systimer->gettime();
					break;

				case cmd_network_get_time:
					end_of_sequence = false;
					more_data = true;
					result->cmd = cmd_network_get_time;
					result->sequence_id = in->sequence_id;
					result->flag = frame_flag_END;
					result->size = 8;
					*(int64_t*)result->data = get_ref_time();
					break;

				case cmd_network_get_freq:
					end_of_sequence = false;
					more_data = true;
					result->cmd = cmd_network_get_freq;
					result->sequence_id = in->sequence_id;
					result->flag = frame_flag_END;
					result->size = 8;
					{
						float tmp[2] = {uplink_freq, downlink_freq};
						memcpy(result->data, tmp, 8);		// dirty fix for 32bit alignment issue.
					}
					break;

				case cmd_network_set_freq:
					if(in->size == 8)
					{
						float tmp[2];
						memcpy(tmp, in->data, 8);		// dirty fix for 32bit alignment issue.
						uplink_freq = tmp[0];
						downlink_freq = tmp[1];
						mtx.set_frequency(uplink_freq, downlink_freq);
						mrx.set_frequency(uplink_freq, downlink_freq);
						uplink_freq.save();
						downlink_freq.save();
					}
					break;
				
				case cmd_network_get_rate:
					end_of_sequence = false;
					more_data = true;
					result->cmd = cmd_network_get_rate;
					result->sequence_id = in->sequence_id;
					result->flag = frame_flag_END;
					result->size = 1;
					result->data[0] = lora_mode > 0.5f ? 0 : 1;
					break;

				case cmd_network_set_rate:
					if (in->size >= 1)
					{
						lora_mode = in->data[0] == 0 ? 1 : 0;
						lora_mode.save();
						reset_sx127x();
					}
					break;

				case cmd_network_get_channels:
				case cmd_network_get_channel_capacity:
				case cmd_network_set_channels:
					busy = true;
					break;

				case cmd_debug_reset_rf:
					reset_sx127x();
					break;
				
				case cmd_debug_reboot_to_bootloader:
				{
					volatile void * bkp = (void*)0x40002850;
					memcpy((void*)bkp, "hello", 6);
					NVIC_SystemReset();
					break;
				}

				default:
					busy = true;
					break;
			};
			
			LOGE("%d\n", in->sequence_id);

			// ACK/NACK
			out->cmd = busy ? cmd_nack : cmd_ack;
			out->size = 0;
			out->sequence_id = in->sequence_id;
			out->flag = (end_of_sequence && !more_data) ? frame_flag_END : 0;
			update_crc(out);

			usb.write(out, out->size + HEADER_SIZE);

			// result?
			if (more_data)
			{
				update_crc(result);
				usb.write(result, result->size + HEADER_SIZE);
			}
		}

		// aux(RTK) uart port forwarding
		times[2] = systimer->gettime();
		if (mtx.txqueue_space(RTK_PRIORITY) && (systimer->gettime() > next_rtk_frame || uart_rtk.available() > RTK_MTU))
		{
			WirelessPacket * wp = (WirelessPacket*) p.data;
			int c = uart_rtk.peak(wp->payload, RTK_MTU);

			if (c>0)
			{
				led_usart_rtk.toggle();

				wp->type = packet_payload_rtk;
				wp->length = c;
				wp->src_node_id = NODE_STATION;
				wp->dest_node_id = NODE_BROADCAST;
				update_crc(wp);
				p.size = PACKET_HEADER_SIZE + wp->length;

				LOGE("RTK:%d bytes\n", wp->length);

				if (0 == mtx.write(p, RTK_PRIORITY))
				{
					led_lora1.toggle();
					next_rtk_frame = systimer->gettime() + RTK_interval;
					uart_rtk.read(wp->payload, c);	// remove from uart buffer
				}
			}
			else
			{
				next_rtk_frame = systimer->gettime() + RTK_interval;
			}
		}		

		// send flying packets
		times[3] = systimer->gettime();
		if (flying_packet_count > 0)
		{
next_flying:
			handle_air_rx(usb, out);
			bool sent = false;
			if (flying_packet_count>0 && flying_packets[0].next_tx < systimer->gettime())
			{
				if (flying_packets[0].retries_left > 0)
				{
					if (mtx.ready_for_next_tx())
						sent = mtx.write(flying_packets[0], 1) == 0;
				}
				else
				{
					uint16_t sid = flying_packets_usb_sequence_id[0];
					//RTT_printf("DROP:%d, ts=%lld\n", flying_packets[0].data[8], systimer->gettime());
					for(int i=0; i<flying_packet_count-1; i++)
						flying_packets[i] = flying_packets[i+1];
					flying_packet_count --;

					// USB result
					out->cmd = cmd_tx_withack;
					out->size = 1;
					out->sequence_id = sid;
					out->flag = frame_flag_END;
					out->data[0] = 0;
					update_crc(out);

					usb.write(out, out->size + HEADER_SIZE);
					goto next_flying;
				}
			}

			if (sent)
			{
				mtx.flush();
				int64_t t = systimer->gettime();
				
				led_lora1.toggle();
				
				sx127x_packet p = flying_packets[0];
				uint16_t sid = flying_packets_usb_sequence_id[0];
				for(int i=0; i<flying_packet_count-1; i++)
					flying_packets[i] = flying_packets[i+1];

				p.retries_left --;
				int payload_bytes_aes = (p.size + 15) & 0xfff0;
				int airtime = (lora_mode>0.5f) ? lora_calc_airtime(500, 1, 7, payload_bytes_aes) : gfsk_calc_airtime(250000, payload_bytes_aes);
				p.next_tx = systimer->gettime() + airtime + MAX_TX_QUEUE_TIME + ((lora_mode>0.5f) ? TX_INTERVAL_LORA : TX_INTERVAL_GFSK) * 2;
				//printf("FLY:%d, ts=%lld, next_ts=%lld\n", p.data[8], systimer->gettime(), p.next_tx);

				flying_packets[flying_packet_count-1] = p;
				flying_packets_usb_sequence_id[flying_packet_count-1] = sid;

				tx ++;
				
				t = systimer->gettime() -t;
				LOGE("XX:%lld\n", t);
			}
		}

		// send probing/timing packets
		// note that timing packet is identical to probing packet with a different packet type.
		times[4] = systimer->gettime();
		if (systimer->gettime() > next_probing)
		{
			WirelessPacket * wp = (WirelessPacket*) p.data;
			wp->length = 8;
			*(int64_t*)wp->payload = get_ref_time();
			wp->type = probe_nodes ? packet_probe : packet_timing;
			wp->src_node_id = NODE_STATION;
			wp->dest_node_id = NODE_BROADCAST;
			update_crc(wp);
			p.size = PACKET_HEADER_SIZE + 8;

			if (mtx.write(p, 1) == 0)
			{				
				if (probing_interval == 0)
				{
					probe_nodes = false;
					probing_interval = 2000000;
				}
				
				next_probing = systimer->gettime() + probing_interval;
				led_lora1.toggle();
			}
		}
		
		// SX127x stuck reset
		if (mtx.stuck() || mrx.stuck())
			reset_sx127x();
		
		// ublox Survey-in packet
		uint8_t survey_in_pkt[] = {0xB5,0x62,0x06,0x71,0x28,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
									0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC8,0x00,0x00,0x00,0x30,0x75,
									0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x80};
		
		static int64_t last_survey_in = 0;
		if (systimer->gettime() > last_survey_in + 5000000)
		{
			uart_rtk.write(survey_in_pkt, sizeof(survey_in_pkt));
			led_usart_rtk.toggle();
			last_survey_in = systimer->gettime();
		}
	
		times[5] = systimer->gettime();
	}
}

