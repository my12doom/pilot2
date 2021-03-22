#include <stdint.h>
#include "ZLink.h"
#include <stdio.h>
#include <string.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4UART2.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4VCP.h>
#include <HAL/aux_devices/LoraRFHelper.h>
#include <HAL/aux_devices/Sx127x.h>
#include <utils/param.h>

#include <modules/utils/space.h>
#include <modules/utils/crc16.h>
#include <modules/utils/SHA1.h>

#include "RGBLED.h"
#include "ZLinkWireless.h"
#include "ZlinkFile.h"
#include "ZLinkManagementPacket.h"
#include "ZLinkWifi.h"
#include "CompressedFile.h"
#include "mavlink.h"

#include <FileSystem/ff.h>
#include "mavlink/hover/mavlink.h"
#include "esp-serial-flasher/esp_loader.h"
#include "esp-serial-flasher/md5_hash.h"
#include "esp-serial-flasher/serial_io.h"
#include "esp-serial-flasher/stub.h"

using namespace HAL;
using namespace STM32F4;
using namespace hover;

#define countof(x) (sizeof(x)/sizeof(x[0]))

// configuration
#define DOWNLINK_PRIORITY 1
#define DOWNLINK_MTU 50
#define DOWNLINK_AGGREGATION_TIME 200000
#define DOWNLINK_TX_INTERVAL_LORA 20000
#define DOWNLINK_TX_INTERVAL_GFSK 3000
#define DOWNLINK_RETRY 5
param lora_mode("lora", 1);
param nodeid("node", -1);
param uplink_freq("fup", 429);
param downlink_freq("fdwn", 437);
param portpower("pwr", 65535);
static uint8_t aes_key[32] = {0x85, 0xA3};
static bool use_aes = false;

// hardware/board
F4GPIO led_uart1(GPIOB, GPIO_Pin_1);
F4GPIO led_uart_rtk(GPIOB, GPIO_Pin_4);
F4GPIO led_wireless(GPIOB, GPIO_Pin_0);
F4GPIO led_usb(GPIOB, GPIO_Pin_5);
STM32F4::F4UART2 uart_payload(USART2);
STM32F4::F4UART2 uart_RTK(USART1);
uint8_t uart_wifi_rx_buf[16384];
STM32F4::F4UART2 uart_wifi(USART6);//
F4Interrupt DIO0;
F4Interrupt DIO1;
IUART *esp_uart = &uart_wifi;
F4GPIO _rst(GPIOC, GPIO_Pin_4);
F4GPIO _io0(GPIOC, GPIO_Pin_5);
IGPIO *esp_io0 = &_io0;
IGPIO *esp_rst = &_rst;

RGBLED rgb;
SX127xManager xk;
F4Timer t3(TIM3);
F4GPIO rst(GPIOC, GPIO_Pin_14);

extern "C" void TIM3_IRQHandler(void)
{
	t3.call_callback();
}

F4Timer t5(TIM5);

extern "C" void TIM5_IRQHandler(void)
{
	t5.call_callback();
}

//
enum
{
	path_sx127x = 0,
	path_tcp = 1,
	path_udp = 2,
	path_drop = 3,
} link_path;


// states
ZLinkNodeState state;
int64_t probe_reponse_ts = -1;
int64_t ref_dt = 0;
int64_t fs_start_time = 0;

int ack_list[10];
int ack_index = 0;

// downlink
sx127x_packet downlink_pkt = {0};
uint16_t downlink_sequence_id = 1;
int64_t next_downlink_time = 0;
int downlink_retry_left;
int downlink_path = path_drop;
file_hash_result_info sha1_for_management = {management_file_hash_result};
bool has_pending_sha1 = false;
uint8_t sha1_result_path = path_drop;
file_decompress_result_info decompress_for_management = {management_file_decompress_result};
bool has_pending_decompress_result = false;
uint8_t decompress_result_path = path_sx127x;

// filesystems
FATFS fs;
FIL f;
bool fs_OK = false;
bool file_ready = false;
uint8_t block_space[sizeof(ZLinkFileBlock)+ZLinkMaxFileBlockPayload] = {0};
ZLinkFileBlock *fblock = (ZLinkFileBlock *)block_space;
#define FR_FAIL_RET(x) if(x != FR_OK) {file_ready = false; return -1;}

// helpers
sx127x_packet p;
sx127x_packet probe_respond_packet;

// mavlink
mavlink_status_t mav_status;
mavlink_message_t mav_msg;
mavlink_message_t mav_msg94;
bool new_94 = false;
int mavlink_chan = 0;

// wifi
uint8_t wifi_buf[sizeof(ZLinkWifiPacket)+MAX_ZLINK_WIFI_PKT_SIZE];
ZLinkWifiPacket* wifi_pkt = (ZLinkWifiPacket*)wifi_buf;
int update_wifi_firmware(uint8_t *tmp, int tmp_size);

int64_t get_ref_time()
{
	return systimer->gettime() + ref_dt;
}

bool port_power(int port_number)
{
	int p = (int)portpower;

	return p&(1<<port_number);
}

int reset_sx127x()
{
	rst.write(0);
	systimer->delayms(1);
	rst.write(1);
	xk.set_lora_mode(lora_mode);

	// magic
	p.size = 20;
	xk.write(p, 0);
	//systimer->delayms(25);
	
	return 0;
}


int fs_init()
{
	if(disk_initialize(0) != FR_OK)
		return -1;
	if (f_mount(&fs, "", 0) != FR_OK)
		return -2;

	fs_OK = true;

	return 0;
}

int filesize = 0;
int fs_closefile()
{
	file_ready = false;
	filesize = f_size(&f);

	return f_close(&f);
}

int fs_openfile(const char *filename, bool for_read)
{
	if (file_ready)
		fs_closefile();
	if ( f_open(&f, filename, for_read ? (FA_READ | FA_OPEN_EXISTING) : (FA_CREATE_ALWAYS | FA_WRITE | FA_READ)) != FR_OK)
		return -1;
	file_ready = true;
	return 0;
}

int fs_writefile(const uint8_t *block, int size)
{
	int rtn = -1;
	if (file_ready)
	{
		UINT bw;
		return f_write(&f, block, size, &bw);
	}
	
	return rtn;
}

int fs_deletefile(const char *filename)
{
	return f_unlink(filename);
}

int fs_sha1(uint8_t *out)
{
	if (!file_ready)
	{
		memset(out, 0, 20);
		return -1;
	}
	int left = f_size(&f);
	f_lseek(&f, 0);
	const int sector_size = 512;
	uint8_t buf[sector_size];
	SHA1_STATETYPE sha1;
	SHA1_Start(&sha1);	

	while(left)
	{
		int block_size = left > sector_size ? sector_size : left;
		FRESULT fr = f_read(&f, buf, block_size, NULL);
		if (fr != 0)
		{
			fs_closefile();
			return -2;
		}

		SHA1_Hash(buf, block_size, &sha1);
		left -= block_size;
	}
	SHA1_Finish(out, &sha1);

	return 0;
}

int64_t time;
uint8_t mavlink_buf[61];

int search_and_execute(FIL *f)
{
	// replay started?
	if (fs_start_time == 0 )
		return 0;

	if (!file_ready)
		return -1;
next:
	// block pending?
	// check timestamp
	if (fblock->size > 0)
	{
		time = get_ref_time() - fs_start_time;
		if (time < fblock->timestamp)
			return 0;

		// execute
		switch(fblock->type)
		{
			case block_uart_payload:
				{
					int rtn = -1;
					if (port_power(0))
						rtn = uart_payload.write(fblock->payload, fblock->size);
					led_uart1.toggle();
					static int64_t lt = systimer->gettime();
					//printf("dt=%d, %d\n", int (systimer->gettime()-lt), rtn);
					//lt = systimer->gettime();
				}
				break;
			case block_RGB_cmd:
				if (port_power(2))
				{
					rgb.write(fblock->payload[0], fblock->payload[1], fblock->payload[2]);
				}
				else
				{
					rgb.write(0,0,0);
				}
				break;

			case block_mavlink_ned:
				{
					int osize = 61;
					if (fblock->size == 12)
					{
						mavlink_header *pm = (mavlink_header*)mavlink_buf;
						pm->magic = 0xFE;
						pm->len = 53;
						pm->msgid = 95;
						pm->seq = 0;
						pm->sysid = 1;

						hover::mavlink_set_position_target_wgs84_local_ned_t * pned = (hover::mavlink_set_position_target_wgs84_local_ned_t*)pm->payload;
						memset(pned, 0, 53);
						memcpy(&pned->x, fblock->payload, 12);
						pned->time_boot_ms = systimer->gettime() / 1000;
						pned->type_mask = 0x0df8;
						pned->target_system = 1;
						pned->target_component = 1;
						pned->coordinate_frame = 1;
						
						uint16_t *pcrc = (uint16_t *)&pm->payload[pm->len];
						pm->payload[pm->len] = 212;					// "CRC_EXTRA" for msgid 95
						*pcrc = hover::crc_calculate(mavlink_buf+1, sizeof(mavlink_header) + pm->len);
						
						if (port_power(0))
						{
							uart_payload.write(pm, 61);
							led_uart1.toggle();
						}
					}
				}
				break;
		}

		fblock->size = 0;
		goto next;
	}


	while(1)
	{
		// check EOF
		if (f_eof(f))
		{
			fs_closefile();
			fs_start_time = 0;
			printf("play ended @ %lld(EOF)\n", get_ref_time());
			return 0;
		}
		
		// search for magic (big endian)0x85A3
		uint32_t pos = f_tell(f);
		FR_FAIL_RET(f_read(f, fblock, 2, NULL));
		if (fblock->magic != 0xA385)
		{
			f_lseek(f, pos + 1);
			continue;
		}

		// check size & crc
		FR_FAIL_RET(f_read(f, &fblock->crc, sizeof(ZLinkFileBlock)-2, NULL));
		if (fblock->size > ZLinkMaxFileBlockPayload)
		{
			f_lseek(f, pos + 2);
			continue;
		}
		FR_FAIL_RET(f_read(f, fblock->payload, fblock->size, NULL));
		uint16_t crc = crc16(&fblock->size, sizeof(ZLinkFileBlock) + fblock->size - 4);

		if (crc == fblock->crc)
			goto next;
		
		f_lseek(f, pos + 2);
	}
}

int fileplay_stop()
{
	fs_start_time = 0;
	fblock->size = 0;

	return 0;
}

int enqueue_downlink(uint8_t *data, int size, int type, uint8_t path)
{
	if (size > 254 - HEADER_SIZE)
		return 0;		// DROPPED!

	if (downlink_pkt.size > 0)
		return -1;
	
	if (path == path_drop)
		return -1;

	printf("downlink:%d bytes, path:%d\n", size, path);

	// generate downlink packet
	downlink_sequence_id++;

	WirelessPacket *wp = (WirelessPacket*)downlink_pkt.data;	
	wp->type = type;
	if (path == path_sx127x)
		wp->type |= packet_flag_need_ack;
	else
		wp->type &= ~packet_flag_need_ack;
	
	wp->length = size;
	wp->src_node_id = nodeid;
	wp->dest_node_id = NODE_STATION;
	wp->sequence_id = downlink_sequence_id;
	memcpy(wp->payload, data, size);
	update_crc(wp);

	
	if (path == path_sx127x)
	{
		downlink_pkt.size = PACKET_HEADER_SIZE + wp->length;
		downlink_retry_left = DOWNLINK_RETRY - 1;
	}


	if (path == path_tcp || path == path_udp)
	{
		// send to uart directly
		fill_pkt(wifi_pkt, path, downlink_pkt.data, PACKET_HEADER_SIZE + wp->length);
		uart_wifi.write((uint8_t*)wifi_pkt, sizeof(ZLinkWifiPacket) + wifi_pkt->size);
	}

	return 0;
}

int delay_next_downlink()
{
	next_downlink_time = systimer->gettime() + (lora_mode> 0.5f? DOWNLINK_TX_INTERVAL_LORA : DOWNLINK_TX_INTERVAL_GFSK);
	return 0;
}

int try_send_next_downlink()
{
	if (downlink_pkt.size == 0)
		return 0;

	if (systimer->gettime() < next_downlink_time)
		return -1;

	if (0 == xk.write(downlink_pkt, DOWNLINK_PRIORITY))
	{
		if (downlink_retry_left)
			downlink_retry_left --;
		else
			downlink_pkt.size = 0;
		
		int airtime = (lora_mode>0.5f) ? lora_calc_airtime(500, 1, 7, downlink_pkt.size) : gfsk_calc_airtime(250000, downlink_pkt.size);
		next_downlink_time = systimer->gettime() + airtime + (lora_mode> 0.5f? DOWNLINK_TX_INTERVAL_LORA : DOWNLINK_TX_INTERVAL_GFSK);
	}

	return 0;
}

int handle_management_packet(uint8_t *packet, int size, uint8_t path)
{
	management_info *m = (management_info*)packet;
	switch(m->service)
	{
		case management_change_nodeid:
			if (size > 4 && nodeid == m->change_nodeid.old_nodeid)
			{
				nodeid = m->change_nodeid.new_nodeid;
				nodeid.save();
			}
			break;
		case management_change_aes_key:
			if (size > 32)
			{
				memcpy(aes_key, m->change_aes.new_key, 32);
				space_write("AES", 3, aes_key, 32, NULL);
				xk.set_aes(aes_key, 32);
			}
			break;
		case management_change_freq:
			uplink_freq = m->change_freq.uplink_freq / 1e6;
			downlink_freq = m->change_freq.downlink_freq / 1e6;
			xk.set_frequency(downlink_freq, uplink_freq);		// note that on nodes, TX is "downlink"
			uplink_freq.save();
			downlink_freq.save();
			break;

		case management_file_open:
			m->file_open.filename[12] = NULL;
			fs_openfile(m->file_open.filename, false);
			break;
		case management_file_block:
			fs_writefile(m->file_block.block, m->file_block.block_size);
			break;
		case management_file_close:
			fs_closefile();
			break;
		case management_file_delete:
			m->file_delete.filename[12] = NULL;
			fs_deletefile(m->file_delete.filename);
			break;
		case management_file_play:
			if (!fs_start_time)
			{
				m->file_play.filename[12] = NULL;
				fs_openfile(m->file_play.filename, true);
				fs_start_time = m->file_play.ref_timestamp;
				time = get_ref_time();

				printf("play start @ %lld\n", fs_start_time);
			}
			else
			{
				printf("play already started\n");
			}			
			break;
			
		case management_file_stop:
			fileplay_stop();
			break;
		case management_RGB:
			if (port_power(2))
				rgb.write(m->RGB.R, m->RGB.G, m->RGB.B);
			break;
		case management_port_power:
			portpower = m->uart_power.power;
			portpower.save();
			if (!port_power(2))
				rgb.write(0,0,0);
			break;
		case management_air_rate:
			lora_mode = m->air_rate.rate == 0 ? 1 : 0;
			lora_mode.save();
			rst.write(0);
			systimer->delayms(1);
			rst.write(1);
			xk.set_lora_mode(lora_mode);
		
			// magic
			p.size = 20;
			xk.write(p, 0);
			systimer->delayms(25);

			break;

		case management_file_hash_request:
			m->file_hash_request.filename[12] = NULL;
			fs_openfile(m->file_hash_request.filename, true);
			fs_sha1(sha1_for_management.sha1);
			strcpy(sha1_for_management.filename, m->file_hash_request.filename);
			has_pending_sha1 = true;
			sha1_result_path = path;
			break;
		case management_file_decompress_request:
			m->file_decompress_request.in_filename[12] = NULL;
			m->file_decompress_request.out_filename[12] = NULL;
			decompress_for_management.success = 
				0 == decompress_file(m->file_decompress_request.in_filename, m->file_decompress_request.out_filename) ? 1 : 0;
			fs_openfile(m->file_decompress_request.out_filename, true);
			fs_sha1(decompress_for_management.sha1);
			fs_closefile();
			strcpy(decompress_for_management.out_filename, m->file_decompress_request.out_filename);
			has_pending_decompress_result = true;
			decompress_result_path = path;
			break;

		case management_reset_system:
			NVIC_SystemReset();
		default:
			break;
	}

	downlink_path = path;

	return 0;
}

// wireless
// source: 0:sx127x, 1:wifi_uart, others: undefined
int handle_wirelesspacket(WirelessPacket* p, uint8_t path, sx127x_packet *pmeta = NULL)
{
	bool crc_ok = check_crc(p);
	
	if (!crc_ok)
		return 0;

	led_wireless.toggle();
	
	static sx127x_packet tx;
	WirelessPacket *ptx = (WirelessPacket *)tx.data;

	if (p->src_node_id == NODE_STATION && pmeta)
		state.rssi_onboard = pmeta->power;

	if (p->dest_node_id == nodeid || p->dest_node_id == NODE_BROADCAST)
	{
		int type = p->type & 0x7f;

		if (type != packet_ack)
			delay_next_downlink();

		if (p->type & packet_flag_need_ack && path == path_sx127x)
		{
			tx.size = PACKET_HEADER_SIZE;

			ptx->type = packet_ack;
			ptx->length = 0;
			ptx->src_node_id = nodeid;
			ptx->dest_node_id = p->src_node_id;
			ptx->sequence_id = p->sequence_id;

			update_crc(ptx);
			
			//systimer->delayus(100);		// workaround for module RF switch

			xk.write(tx, 0);
			xk.flush();
			
			for(int i=0; i<countof(ack_list); i++)
				if (ack_list[i] == p->sequence_id)
					return 0;
			
			ack_list[ack_index] = p->sequence_id;
			ack_index = (ack_index+1)%countof(ack_list);
		}

		switch(type)
		{
			case packet_ack:
				if (p->sequence_id == downlink_sequence_id)
					downlink_pkt.size = 0;
				break;

			case packet_payload:
				if (port_power(0))
				{
					uart_payload.write(p->payload, p->length);
					led_uart1.toggle();
				}
				if (path != path_sx127x)
					downlink_path = path;
				break;

			case packet_payload_rtk:
				if (port_power(1))
				{
					uart_RTK.write(p->payload, p->length);
					led_uart_rtk.toggle();
				}
				break;

			case packet_management:
				handle_management_packet(p->payload, p->length, path);
				break;

			case packet_timing:
				// synchronize timing
				// only use timing/probing packet from sx127x
				if (path == path_sx127x && p->length == 8)
					ref_dt = *(int64_t*)p->payload - systimer->gettime();
				break;

			case packet_probe:
				// synchronize timing
				// only use timing/probing packet from sx127x
				if (path == path_sx127x && p->length == 8)
					ref_dt = *(int64_t*)p->payload - systimer->gettime();

				ptx->type = packet_probe_respond;
				ptx->length = sizeof(ZLinkNodeState);
				ptx->src_node_id = nodeid;
				ptx->dest_node_id = p->src_node_id;
				ptx->sequence_id = p->sequence_id;
				state.nodeid = nodeid;
				memcpy(ptx->payload, &state, sizeof(ZLinkNodeState));

				update_crc(ptx);

				// schedual for sx127x or directly for TCP/UDP
				if(path == path_sx127x)
				{
					tx.size = PACKET_HEADER_SIZE + sizeof(ZLinkNodeState);
					probe_respond_packet = tx;
					probe_reponse_ts = systimer-> gettime() + DOWNLINK_TX_INTERVAL_LORA * (nodeid-1);
				}
				else
				{
					fill_pkt(wifi_pkt, path, (uint8_t*)ptx, PACKET_HEADER_SIZE + sizeof(ZLinkNodeState));
					uart_wifi.write((uint8_t*)wifi_pkt, sizeof(ZLinkWifiPacket) + wifi_pkt->size);
				}

				break;
		}
	}

	return 0;
}

void fs_cb(void *p)
{
	search_and_execute(&f);
}

uint8_t usb_space[MAX_USB_DATA_SIZE*3 + 3*sizeof(ZLinkUSBFrame)];

int handle_usb(F4VCP &usb)
{
	static ZLinkUSBFrame *in = (ZLinkUSBFrame *)usb_space;
	static ZLinkUSBFrame *out = (ZLinkUSBFrame *)(usb_space + MAX_USB_DATA_SIZE + sizeof(ZLinkUSBFrame));
	static ZLinkUSBFrame *result = (ZLinkUSBFrame *)(usb_space + 2*(MAX_USB_DATA_SIZE + sizeof(ZLinkUSBFrame)));
	
	int c = search_frame(&usb, in);
	if (c>0)
	{
		led_usb.toggle();
		bool busy = false;
		bool more_data = false;
		result->size = 0;	// 0: no result

		switch (in->cmd)
		{
			case cmd_node_set_aes:
				if(in->size == 32)
					memcpy(aes_key, in->data, in->size);
				xk.set_aes(aes_key, 32);
				space_write("AES", 3, aes_key, 32, NULL);
				break;

			case cmd_node_set_id:
				if(in->size == 2)
				{
					nodeid = *(uint16_t*)in->data;
					nodeid.save();
				}
				break;

			case cmd_node_get_id:
				more_data = true;
				result->cmd = cmd_node_get_id;
				result->sequence_id = in->sequence_id;
				result->flag = frame_flag_END;
				result->size = 2;
				*(uint16_t*)result->data = nodeid;
				break;
			
			case cmd_network_get_freq:
				more_data = true;
				result->cmd = cmd_network_get_freq;
				result->sequence_id = in->sequence_id;
				result->flag = frame_flag_END;
				result->size = 8;
				{
					float tmp[2] = {uplink_freq, downlink_freq};
					memcpy(result->data, tmp, 8);		// dirty fix for FPU 32bit alignment issue.
				}
				break;

			case cmd_network_set_freq:
				if(in->size == 8)
				{
					float tmp[2];
					memcpy(tmp, in->data, 8);			// dirty fix for FPU 32bit alignment issue.
					uplink_freq = tmp[0];
					downlink_freq = tmp[1];
					xk.set_frequency(downlink_freq, uplink_freq);		// note that on nodes, TX is "downlink"
					uplink_freq.save();
					downlink_freq.save();
				}
				break;
			
			case cmd_network_get_rate:
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
				}
				break;
				
			case cmd_debug_reboot_to_bootloader:
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
				void * bkp = (void*)0x40002850;
				memcpy(bkp, "hello", 6);
				NVIC_SystemReset();
				break;
			}

			default:
				busy = true;
				break;
		};
		
		// ACK/NACK
		out->cmd = busy ? cmd_nack : cmd_ack;
		out->size = 0;
		out->sequence_id = in->sequence_id;
		out->flag = !more_data ? frame_flag_END : 0;
		update_crc(out);

		usb.write(out, out->size + HEADER_SIZE);

		// result?
		if (more_data)
		{
			update_crc(result);
			usb.write(result, result->size + HEADER_SIZE);
		}
	}
	
	return 0;
}

int main()
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	bool wifi_ok = ESP_LOADER_SUCCESS == update_wifi_firmware(uart_wifi_rx_buf, sizeof(uart_wifi_rx_buf));
	uart_wifi.set_buffer_override(0,0, uart_wifi_rx_buf, sizeof(uart_wifi_rx_buf));
	
	//F4VCP vcp;
		
	if (nodeid == -1)
	{
		nodeid = 1;
		nodeid.save();
	}
	
	// load aes keys and init AES codec
	space_read("AES", 3, aes_key, 32, NULL);
	xk.set_aes(aes_key, 32);

	// filesystem helpers
	fs_init();
	fs_openfile("tb.bin", true);

	
	t3.set_period(50000);
	t3.set_callback(fs_cb, NULL);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_Init(&NVIC_InitStructure);
	
	// init SX1278
	rst.set_mode(MODE_OUT_PushPull);
	rst.write(0);
	systimer->delayms(1);
	rst.write(1);

	F4GPIO nss(GPIOA, GPIO_Pin_4);
	F4SPI spi(SPI1);
	nss.set_mode(MODE_OUT_PushPull);
	DIO0.init(GPIOA, GPIO_Pin_8, interrupt_rising);
	DIO1.init(GPIOA, GPIO_Pin_15, interrupt_rising_or_falling);

	SX127x x;
	if (x.init(&spi, &nss, NULL, NULL)<0 && !wifi_ok)
	{
		led_wireless.set_mode(MODE_OUT_PushPull);
		int v = 0;
		while(1)
		{
			led_wireless.toggle();
			systimer->delayms(1500);
			
			v++;
			rgb.write((v&1)?255:0, (v&2)?255:0, (v&4)?255:0);
		}
	}
	
	rgb.write(255, 255, 255);
	
	// init things
	for(int i=0; i<countof(ack_list); i++)
		ack_list[i] = -1;

	led_wireless.set_mode(MODE_OUT_PushPull);
	led_uart1.set_mode(MODE_OUT_PushPull);
	led_uart_rtk.set_mode(MODE_OUT_PushPull);
	led_usb.set_mode(MODE_OUT_PushPull);
	led_wireless.write(1);
	led_uart1.write(1);
	led_uart_rtk.write(1);
	led_usb.write(1);
	uart_payload.set_baudrate(57600);
	uart_RTK.set_baudrate(57600);
	uart_wifi.set_baudrate(1000000);

	x.set_lora_mode(lora_mode);
	
	xk.init(&x, &DIO0, &t5, &DIO1);
	xk.set_frequency(downlink_freq, uplink_freq);		// note that on nodes, TX is "downlink"
	xk.set_tx_interval(1000);
	xk.set_aes(aes_key, 32);


	// magic
	for(int i=0; i<200; i++)
		p.data[i] = i;
	p.size = 200;
	xk.write(p, 0);
	systimer->delayms(100);


	/*
	char tx[512];
	for(int i=0; i<sizeof(tx); i++)
		tx[i] = i^~i;

	while(1)
	{
		uart_wifi.write(tx, sizeof(tx));
		char tmp[1024];
		
		systimer->delayms(10);

		int c = uart_wifi.read(tmp, sizeof(tmp));
		if (c != sizeof(tx) || memcmp(tmp, tx, sizeof(tx)))
		{
			printf("wifi_uart loopback error:%d/%d", sizeof(tx), c);;
		}

		if (c>0)
			uart_wifi.write(tmp, c);
	}
	*/
	
	// sensitivity test
	/*
	xk.set_frequency(437, 437);
	xk.set_lora_mode(true);
	while (1)
	{
		#if 0
		p.size = 50;
		WirelessPacket *wp = (WirelessPacket*)p.data;
		wp->length = 40;
		update_crc(wp);
		mtx.write(p, 0);
		#endif
		
		systimer->delayms(100);
		static sx127x_packet pr;
		WirelessPacket *wpr = (WirelessPacket*)pr.data;
		if (xk.read(&pr) == 0)
		{
			led_wireless.toggle();
			printf("%dRX:%d\n", pr.power, check_crc(wpr) ? 1 : 0);
		}
	}
	*/
	
	int rx = 0;
	int64_t l = systimer->gettime();
	
	int64_t last_tx = systimer->gettime();
	int drop = 0;
	int count = 0;
	int64_t tick = 0;
	while(1)
	{
		// downlink test
		/*
		if (systimer->gettime() > tick + 1000000)
		{
			tick = systimer->gettime();
			char tmp[40];
			sprintf(tmp, "HelloWorld%d", int(tick));
			enqueue_downlink((uint8_t*)tmp, strlen(tmp));
		}
		*/
		
		// stuck recovery
		int s = xk.stuck();
		if (s)
		{
			printf("reset sx127x, reason=%d\n", s);
			reset_sx127x();
		}

		// handle RX packets
		sx127x_packet p;
		if (xk.read(&p) == 0)
		{
			count ++;
			int dt= systimer->gettime()-last_tx;
			last_tx = systimer->gettime();
			if (dt>50000)
				drop ++;
			//printf("RX:%d, dt=%d, drop=%d/%d\n", p.size, dt, drop, count);
			handle_wirelesspacket((WirelessPacket*)p.data, path_sx127x, &p);
		}
		
		// handle wifi packets
		static int64_t last = systimer->gettime();
		static int64_t max_dt = 0;
		int64_t dt = systimer->gettime() - last;
		if (dt > max_dt)
			max_dt = dt;
		last = systimer->gettime();
		while (search_pkt(&uart_wifi, wifi_pkt) > 0)
		{
			int64_t t = systimer->gettime();
			if (wifi_pkt->type == path_tcp || wifi_pkt->type == path_udp)
				handle_wirelesspacket((WirelessPacket*)wifi_pkt->data, wifi_pkt->type);
			int dt = systimer->gettime() - t;
			if (dt > 20000)
				printf("\nhandle=%dus\n", dt);
			
			static int pkt_count = 0;
			printf("\r%d:%d bytes, dt=%dus", ++pkt_count, wifi_pkt->size, int(max_dt));
		}

		// send delayed probe reponse
		if (probe_reponse_ts>0 && systimer->gettime() > probe_reponse_ts)
		{
			if (xk.write(probe_respond_packet, 1) == 0)
				probe_reponse_ts = -1;
		}

		// try send downlink
		try_send_next_downlink();
		
		// drain uart
		int c = -1;
		do
		{
			uint8_t tmp[DOWNLINK_MTU];
			c = uart_payload.read(tmp, DOWNLINK_MTU);
			for(int i=0; i<c; i++)
			{
				if (mavlink_parse_char(mavlink_chan, tmp[i], &mav_msg, &mav_status))
				{				
					led_uart1.toggle();
					if (mav_msg.msgid == 94)
					{
						mav_msg94 = mav_msg;
						new_94 = true;
					}
				}
			}
		} while(c>0);
		
		if (new_94 && port_power(3))
		{
			WirelessPacket * wp = (WirelessPacket*) p.data;
			int mavlink_buf_size = mavlink_msg_to_send_buffer(wp->payload, &mav_msg94);					
			if (enqueue_downlink(wp->payload, mavlink_buf_size, packet_payload, downlink_path) == 0)
				new_94 = false;
		}

		/*
		int c = uart_payload.peak(wp->payload, DOWNLINK_MTU);
		if (port_power(0) && c>0)
		{
			led_uart1.toggle();

			if (0 == enqueue_downlink(wp->payload, c))
				uart_payload.read(wp->payload, c);	// remove from uart buffer
		}
		*/

		// sha1 downlink
		if (has_pending_sha1)
		{
			if (0 == enqueue_downlink((uint8_t*)&sha1_for_management, sizeof(sha1_for_management), packet_result, sha1_result_path))
				has_pending_sha1 = false;
		}

		// decompress result downlink
		if (has_pending_decompress_result)
		{
			if (0 == enqueue_downlink((uint8_t*)&decompress_for_management, sizeof(decompress_for_management), packet_result, decompress_result_path))
				has_pending_decompress_result = false;
		}


		// timing PPS indicator
		led_usb.write(get_ref_time() % 1000000 > 200000);

		// USB
		//handle_usb(vcp);
		
		// debug: print lna gain
		//printf("\rLNA gain: %d\n   ", x.read_reg(0x0c)>>5);
	}
}








int load_baud = 2000000;
bool use_stub = true;
bool forced_flash = false;

int sync_baud = 115200;
int ram_block_size = 0x200;
int stub_flash_block_size = 0x4000;
int app_offset = 0x10000;

esp_loader_error_t load_stub()
{
	for (int i = 0; i < 2; i++)
	{
		int ssize = i == 0 ? stub_text_size : stub_data_size;
		uint8_t *p = (uint8_t *)(i == 0 ? stub_text : stub_data);
		uint32_t addr = i == 0 ? stub_text_addr : stub_data_addr;

		esp_loader_error_t e = esp_loader_memory_start(addr, ssize, ram_block_size);
		if (e != ESP_LOADER_SUCCESS)
			return e;

		int w = 0;
		while (ssize > 0)
		{
			int s = ssize > ram_block_size ? ram_block_size : ssize;
			e = esp_loader_memory_write(p, s);

			if (e != ESP_LOADER_SUCCESS)
				return e;
			p += s;
			ssize -= s;
			w += s;
		}
	}

	esp_loader_error_t e = esp_loader_memory_finish(0, stub_entry);
	if (e != ESP_LOADER_SUCCESS)
		return e;

	systimer->delayms(100);

	// claer "OHAI" buf
	char tmp[20];
	uart_wifi.read(tmp, sizeof(tmp));

	return e;
}

void clear_wifi_rx_buf()
{
	char tmp[100];
	while (uart_wifi.available() > 0)
		uart_wifi.read(tmp, sizeof(tmp));
}

esp_loader_error_t e = ESP_LOADER_ERROR_FAIL;
struct MD5Context s;
struct MD5Context s_md5_context;
struct MD5Context s2;
int update_wifi_firmware(uint8_t *tmp, int tmp_size)
{
	bool is_stub = false;
	uint8_t file_md5[16];
	uint8_t uart_md5[16];
	uint8_t flash_md5[16];

	UINT got;
	esp_loader_connect_args_t connect_config = { 100,10 };
	int size;
	int block_size;
	int64_t start_time = systimer->gettime();

	// calculate file md5
	if (1)
	{
		if (fs_init() < 0)
			goto end;
		
		if (fs_openfile("8266.bin", true) < 0)
			goto end;

		size = f_size(&f);
		printf("fsize=%d\n", size);
		size = (size + tmp_size - 1) / tmp_size * tmp_size;

		MD5Init(&s_md5_context);
		for(int i=0; i<size / tmp_size; i++)
		{
			memset(tmp, 0xff, tmp_size);
			f_read(&f, tmp, tmp_size, &got);

			// exclude non-app part.
			if (i*tmp_size >= app_offset)
				MD5Update(&s_md5_context, tmp, tmp_size);
		}
		MD5Final(file_md5, &s_md5_context);
		printf("file MD5:");
		for (int i = 0; i < 16; i++)
			printf("%02x", file_md5[i]);

		printf("\n");
	}


	// sync
	uart_wifi.set_baudrate(sync_baud);
	loader_port_enter_bootloader();
	systimer->delayms(200);
	clear_wifi_rx_buf();
	

	e = esp_loader_connect(&connect_config);

	if (e != ESP_LOADER_SUCCESS)
		goto end;
	
	// load stub
	if (use_stub && !is_stub)
	{
		printf("loading stub...");
		e = load_stub();
		printf("%s\n", e == ESP_LOADER_SUCCESS ? "OK" : "ERROR");
		if (e != ESP_LOADER_SUCCESS)
			goto end;

		systimer->delayms(500);

		is_stub = true;
	}
	
	if (is_stub)
	{
		// change baudrate
		printf("change baud rate to %d....", load_baud);
		e = esp_loader_change_baudrate(load_baud, sync_baud);

		printf("%s\n", e == ESP_LOADER_SUCCESS ? "OK" : "ERROR");
		if (e != ESP_LOADER_SUCCESS)
			goto end;

		uart_wifi.set_baudrate(load_baud);
		systimer->delayms(15);
		
		// check app part MD5
		uint8_t md5[16];
		e = esp_loader_read_md5(md5, app_offset, size-app_offset);
		if (e == ESP_LOADER_SUCCESS && memcmp(md5, file_md5, 16) == 0)
		{
			printf("md5 matched!\n");
			if (!forced_flash)
				goto end;
		}
	}
	
	// erase
	printf("erasing....");
	block_size = is_stub ? tmp_size : 4096;
	e = esp_loader_flash_start(0, size, is_stub ? 0x4000 : block_size);
	printf("%s\n", e == ESP_LOADER_SUCCESS ? "OK" : "ERROR");
	if (e != ESP_LOADER_SUCCESS)
		goto end;

	// load
reflash:
	f_lseek(&f, 0);
	MD5Init(&s_md5_context);

	for(int i=0; i<size / block_size; i++)
	{
		memset(tmp, 0xff, block_size);
		f_read(&f, tmp, block_size, &got);
        if (i*tmp_size >= app_offset)
            MD5Update(&s_md5_context, tmp, block_size);

		e = esp_loader_flash_write(tmp, block_size);

		if (e != ESP_LOADER_SUCCESS)
			goto end;

		printf(".");
	}

	MD5Final(uart_md5, &s_md5_context);
	printf("UART MD5:");
	for (int i = 0; i < 16; i++)
		printf("%02x", file_md5[i]);

	printf("\n");
	
	// check flash MD5
	e = esp_loader_read_md5(flash_md5, app_offset, size-app_offset);
	if (e == ESP_LOADER_SUCCESS && memcmp(flash_md5, file_md5, 16) != 0)
	{
		printf("flash md5 mismatch!\n");
		//goto reflash;
	}
	
	e = esp_loader_flash_finish(true);


end:
	// always run target
	loader_port_reset_target();
	uart_wifi.set_baudrate(1000000);
	
	printf("time cost:%d\n", int(systimer->gettime() - start_time)/1000);
	return e;
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
