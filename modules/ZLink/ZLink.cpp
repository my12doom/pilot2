#include "ZLink.h"
#include <string.h>
#include <modules/utils/crc16.h>
#include <stdlib.h>
#include "ZLinkManagementPacket.h"


void update_crc(ZLinkUSBFrame * frame)
{
	frame->header = ZLinkHeader;
	frame->crc = crc16((unsigned char*)&(frame->size), HEADER_SIZE - 4 + frame->size);
	uint16_t crc = crc16((unsigned char*)&(frame->size), HEADER_SIZE - 4 + frame->size);

}

bool check_crc(ZLinkUSBFrame *frame)
{
	uint16_t crc = crc16((unsigned char*)&(frame->size), HEADER_SIZE - 4 + frame->size);

	if (crc != frame->crc)
		return false;

	return (frame->header == ZLinkHeader) && (frame->crc == crc);
}

int search_frame(HAL::IUART *uart, ZLinkUSBFrame *frame)
{
	uint8_t header0 = ZLinkHeader & 0xff;
	uint8_t header1 = (ZLinkHeader >> 8) & 0xff;

	while (uart->available() >= HEADER_SIZE)
	{
		// check head
		uint8_t header[2];
		uart->peak(header, 2);
		if (header[0] != header0 || header[1] != header1)
		{
			uart->read(header, 1);
			continue;
		}

		// check body size
		uart->peak(frame, HEADER_SIZE);
		if (frame->size > MAX_USB_DATA_SIZE)
		{
			uart->read(header, 2);
			continue;
		}

		if (uart->available() < HEADER_SIZE + frame->size)
			return -1;
		
		uart->peak(frame, HEADER_SIZE + frame->size);
		if (check_crc(frame))
		{
			uart->read(frame, HEADER_SIZE + frame->size);
			return HEADER_SIZE + frame->size;
		}
		else
		{
			uart->read(header, 2);
		}
	}

	return 0;
}

ZLinkUSBFrame *new_frame(uint16_t data_size, uint16_t sequence_id, uint8_t cmd, uint8_t flag, const uint8_t *data)
{
	ZLinkUSBFrame * p = (ZLinkUSBFrame*)malloc(data_size + HEADER_SIZE);
	if (!p)
		return NULL;

	p->size = data_size;
	p->sequence_id = sequence_id;
	p->cmd = (ZLinkUSBCmds)cmd;
	p->flag = flag;
	if (data)
		memcpy(p->data, data, data_size);
	update_crc(p);

	return p;
}

ZLinkUSBFrame *clone_frame(ZLinkUSBFrame *frame)
{
	ZLinkUSBFrame * p = (ZLinkUSBFrame*)malloc(frame->size + HEADER_SIZE);
	if (!p)
		return NULL;

	memcpy(p, frame, frame->size + HEADER_SIZE);

	return p;
}

void free_frame(ZLinkUSBFrame *p)
{
	if (p)
		free(p);
}

#ifdef _WIN32

ZLinkUSB::ZLinkUSB()
{
	downlink_cb = NULL;
	sequence_id = 0;
	timeout = 1000;
	uart = NULL;
	thread_exit = false;
	memset(sequence_valid, 0, sizeof(sequence_valid));

	InitializeCriticalSection(&cs);
	InitializeCriticalSection(&cs2);
	InitializeCriticalSection(&cs3);

}

ZLinkUSB::~ZLinkUSB()
{
	disconnect();
	DeleteCriticalSection(&cs);
	DeleteCriticalSection(&cs2);
	DeleteCriticalSection(&cs3);
}

int ZLinkUSB::connect(HAL::IUART *uart)
{
	this->uart = uart;
	uart->set_baudrate(115200);		// dummy
	thread_exit = false;

	h_rx_thread = CreateThread(NULL, 0, rx_thread_entry, this, 0, 0);
	h_async_thread = CreateThread(NULL, 0, async_thread_entry, this, 0, 0);

	// send a nop packet to clear any invalid buffer
	ZLinkUSBFrame *f = new_frame(MAX_USB_DATA_SIZE, sequence_id-1, cmd_nop, 0, NULL);
	uart->write(f, MAX_USB_DATA_SIZE + HEADER_SIZE);

	return 0;
}

int ZLinkUSB::disconnect()
{
	thread_exit = true;
	ResumeThread(h_async_thread);
	WaitForSingleObject(h_rx_thread, INFINITE);
	WaitForSingleObject(h_async_thread, INFINITE);
	uart = NULL;

	EnterCriticalSection(&cs);
	for (std::vector<ZLinkUSBFrame*>::iterator i = rx_frames.begin(); i != rx_frames.end(); ++i)
		free_frame(*i);
	rx_frames.clear();
	LeaveCriticalSection(&cs);

	EnterCriticalSection(&cs2);
	memset(sequence_valid, 0, sizeof(sequence_valid));
	LeaveCriticalSection(&cs2);

	return 0;
}

DWORD ZLinkUSB::rx_thread()
{
	ZLinkUSBFrame * p = new_frame(MAX_USB_DATA_SIZE, 0, 0, 0, NULL);
	while (!thread_exit)
	{
		int count = search_frame(uart, p);
		if (count > 0)
		{
			//printf("RX:0x%02x, scid=%04x, size=%d, %d\n", p->cmd, p->sequence_id, p->size, timeGetTime());
			if (is_valid_sequence(p->sequence_id, true))
			{
				EnterCriticalSection(&cs);
				rx_frames.push_back(clone_frame(p));
				LeaveCriticalSection(&cs);
			}

			if (is_valid_sequence(p->sequence_id, false))
			{
				EnterCriticalSection(&cs3);
				async_frames.push_back(clone_frame(p));
				LeaveCriticalSection(&cs3);

				ResumeThread(h_async_thread);
			}
		}
		else
		{
			Sleep(1);
		}
	}

	free_frame(p);

	return 0;
}

int ZLinkUSB::set_downlink_handler(RPC_callback cb)
{
	EnterCriticalSection(&cs3);
	downlink_cb = cb;
	LeaveCriticalSection(&cs3);

	return 0;
}

DWORD ZLinkUSB::async_thread()
{
	while (!thread_exit)
	{
		ZLinkUSBFrame * f = NULL;

		EnterCriticalSection(&cs3);		
		if (!async_frames.empty())
		{
			f = *async_frames.begin();
			async_frames.erase(async_frames.begin());
		}
		LeaveCriticalSection(&cs3);


		if (f)
		{
			sequence s = get_sequence(f->sequence_id);
			if (s.cb)
				s.cb(f, s.p);
			else if (/*f->sequence_id == SEQUENCE_DOWNLINK ||*/ f->cmd == cmd_rx)
			{
				EnterCriticalSection(&cs3);
				RPC_callback cb = downlink_cb;
				LeaveCriticalSection(&cs3);

				if (cb)
					cb(f, s.p);
			}

			if (f->flag & frame_flag_END)
				end_sequence(f->sequence_id);
			free_frame(f);
			continue;
		}



		SuspendThread(GetCurrentThread());
	}	

	return 0;
}

// synchronized RPC
// use this ONLY FOR fast RPCs like configuration and udp-like TX
int ZLinkUSB::RPC(uint8_t cmd, const uint8_t *data, int data_count, uint8_t *out_data/* = NULL*/, int *out_data_count/* = NULL*/)
{
	ZLinkUSBFrame *p = new_frame(data_count, sequence_id, cmd, 0, data);
	start_sequence(sequence_id, true, timeout);
	sequence_id = (sequence_id + 1) % SEQUENCE_MAX;
	char tmp[1000];
	memcpy(tmp, p, p->size + HEADER_SIZE);
	uart->write(p, p->size + HEADER_SIZE);
	//printf("RPC, cmd=%02x, %d\n", cmd, timeGetTime());

	// wait for ACK / result
	DWORD time = GetTickCount() + timeout;
	int rtn = error_timeout;
	while (GetTickCount() < time)
	{
		bool sleep = true;
		EnterCriticalSection(&cs);
		for (auto i = rx_frames.begin(); i != rx_frames.end();)
		{
			//printf("check:seq%d/%d, %02x\n", (*i)->sequence_id, p->sequence_id, (*i)->cmd);

			if ((*i)->sequence_id == p->sequence_id)
			{
				sleep = false;
				if ((*i)->cmd == cmd_ack)
				{
					rtn = (*i)->flag & frame_flag_END ? error_OK : error_result_timeout;
					if (out_data_count)
						*out_data_count = 0;
				}

				if ((*i)->cmd == cmd_nack)
				{
					free_frame(*i);
					i = rx_frames.erase(i);
					end_sequence(p->sequence_id);
					free_frame(p);
					LeaveCriticalSection(&cs);
					return error_nack;
				}

				if ((*i)->flag & frame_flag_END)
				{
					if (i[0]->cmd == cmd)
						rtn = error_OK;
					if (out_data)
						memcpy(out_data, (*i)->data, (*i)->size);
					if (out_data_count)
						*out_data_count = (*i)->size;
					free_frame(*i);
					i = rx_frames.erase(i);
					end_sequence(p->sequence_id);
					free_frame(p);
					LeaveCriticalSection(&cs);
					return rtn;
				}

				free_frame(*i);
				i = rx_frames.erase(i);
			}
			else
			{
				++i;
			}
		}
		
		LeaveCriticalSection(&cs);

		if (sleep)
			Sleep(1);
	}

	end_sequence(p->sequence_id);
	free_frame(p);
	return rtn;
}

// async RPC
int ZLinkUSB::RPC_async(uint8_t cmd, const uint8_t *data, int data_count, bool wait_ack /* = false */, RPC_callback cb/* = NULL*/, const void *userdata/* = NULL*/)
{
	start_sequence(sequence_id, false, timeout, cb, userdata);
	ZLinkUSBFrame *p = new_frame(data_count, sequence_id, cmd, 0, data);

	sequence_id = (sequence_id + 1) % SEQUENCE_MAX;

	int o = uart->write(p, p->size + HEADER_SIZE);

	free_frame(p);

	return o;
}

int ZLinkUSB::set_timeout(int milliseconds)
{
	timeout = milliseconds;
	return 0;
}

void ZLinkUSB::start_sequence(uint16_t sequence_id, bool sync, DWORD timeout, RPC_callback cb/* = NULL*/, const void *userdata/* = NULL*/)
{
	sequence s = {cb, userdata, sequence_id, GetTickCount() + timeout, sync};

	EnterCriticalSection(&cs2);
	sequence_valid[sequence_id] = true;
	on_going_sequences[sequence_id] = s;
	LeaveCriticalSection(&cs2);
}
void ZLinkUSB::end_sequence(uint16_t sequence_id)
{
	EnterCriticalSection(&cs2);
	sequence_valid[sequence_id] = false;
	LeaveCriticalSection(&cs2);
}

bool ZLinkUSB::is_valid_sequence(uint16_t sequence_id, bool sync)
{
	if (sequence_id == SEQUENCE_DOWNLINK && !sync)
		return true;

	EnterCriticalSection(&cs2);
	bool rtn = on_going_sequences[sequence_id].sync == sync && sequence_valid[sequence_id];
	LeaveCriticalSection(&cs2);

	return rtn;
}

sequence ZLinkUSB::get_sequence(uint16_t sequence_id)
{
	sequence o = { 0, 0, 0, 0, 0 };
	EnterCriticalSection(&cs2);
	o = on_going_sequences[sequence_id];
	LeaveCriticalSection(&cs2);

	return o;
}

bool ZLinkUSB::is_node()
{
	uint16_t id = 0;
	int count = 0;
	int o = RPC(cmd_node_get_id, NULL, 0, (uint8_t*)&id, &count);
	if (o == cmd_nack)
		return false;
	if (o < 0)
		return false;

	return true;
}

int ZLinkUSB::node_get_id(uint16_t *id)
{
	int count = 0;
	int o = RPC(cmd_node_get_id, NULL, 0, (uint8_t*)id, &count);

	if (o < 0)
		return o;

	if (count != 2)
		return error_unknown;
	
	return o;
}

int ZLinkUSB::node_set_id(uint16_t new_id)
{
	return RPC(cmd_node_set_id, (uint8_t*)&new_id, 2, NULL, NULL);
}

int ZLinkUSB::node_set_aes(const uint8_t *new_key)
{
	return RPC(cmd_node_set_aes, new_key, 32, NULL, NULL);
}

int ZLinkUSB::set_freq(float uplink, float downlink)
{
	float tmp[2] = { uplink, downlink };
	return RPC(cmd_network_set_freq, (uint8_t*)tmp, 8, NULL, NULL);
}
int ZLinkUSB::get_freq(float *uplink, float *downlink)
{
	float tmp[2];
	int count = 0;
	int o = RPC(cmd_network_get_freq, NULL, 0, (uint8_t*)tmp, &count);

	if (o < 0)
		return o;

	if (count != 8)
		return error_unknown;

	if (uplink)
		*uplink = tmp[0];
	if (downlink)
		*downlink = tmp[1];

	return o;
}




int ZLink::unicast_noack(int nodeid, const uint8_t *data, int size)
{
	WirelessPacket *p = new_packet(size, 0, packet_payload, NODE_STATION, nodeid, data);
	int o = error_nack;
	do
	{
		o = RPC(cmd_tx_oneshot, (uint8_t*)p, PACKET_HEADER_SIZE + size);
	} while (o == error_nack);

	free_packet(p);

	return o;
}

int ZLink::tx_raw(const uint8_t *data, int size)
{
	int o = error_nack;
	do
	{
		o = RPC(cmd_tx_raw, (uint8_t*)data, size);
	} while (o == error_nack);

	return o;
}

int ZLink::broadcast(const uint8_t *data, int size)
{
	return unicast_noack(NODE_BROADCAST, data, size);
}

int ZLink::unicast_with_ack(int nodeid, const uint8_t *data, int size, int max_retry, bool blocking, uint8_t type)
{
	WirelessPacket *p = new_packet(size, 0, type, NODE_STATION, nodeid, data);
	int o = unicast_with_ack(p, max_retry, blocking);
	free_packet(p);

	return o;
}

int ZLink::unicast_with_ack(WirelessPacket *p, int max_retry, bool blocking)
{

	if (p->dest_node_id == NODE_BROADCAST)
	{
		int o = error_nack;
		do
		{
			o = RPC(cmd_tx_oneshot, (uint8_t*)p, PACKET_HEADER_SIZE + p->length);
		} while (o == error_nack);
		return o;
	}

	p->type |= packet_flag_need_ack;
	update_crc(p);

	uint8_t *p2 = new uint8_t[PACKET_HEADER_SIZE + p->length + 2];
	memcpy(p2, p, PACKET_HEADER_SIZE + p->length);
	p2[PACKET_HEADER_SIZE + p->length] = max_retry;
	p2[PACKET_HEADER_SIZE + p->length + 1] = blocking;

	int o = error_nack;
	int osize = 0;
	uint8_t result[10];
	do
	{
		o = RPC(cmd_tx_withack, p2, PACKET_HEADER_SIZE + p->length + 2, result, &osize);
	} while (o == error_nack);

	delete p2;

	if (osize > 0 && result[0] == 0)
		return error_fail;

	return o;
}

int ZLink::reset_node(int nodeid)
{
	uint8_t service = management_reset_system;
	return unicast_with_ack(nodeid, &service, 1, 5, true, packet_management);
}


int ZLink::upload_file(int nodeid, const char *file, const char *onboard_filename)
{
	int MTU = 200;

	FILE * f = fopen(file, "rb");
	fseek(f, 0, SEEK_END);
	int size = ftell(f);
	int file_size = size;
	uint8_t *data = new uint8_t[size];
	fseek(f, 0, SEEK_SET);
	fread(data, 1, size, f);
	fclose(f);

	// open on board file
	int upload_time = timeGetTime();
	management_info *pkt = (management_info *)malloc(1024);
	pkt->service = management_file_open;
	strcpy(pkt->file_open.filename, onboard_filename);
	if (unicast_with_ack(nodeid, (uint8_t*)pkt, 14, 5, true, packet_management) < 0)
		return -1;

	// send data
	uint8_t *p = data;
	int t2 = timeGetTime();
	while (size)
	{
		int block_size = size > MTU ? MTU : size;
		pkt->service = management_file_block;
		pkt->file_block.block_size = block_size;
		memcpy(pkt->file_block.block, p, block_size);
		int t = timeGetTime();
		int o = unicast_with_ack(nodeid, (uint8_t*)pkt, block_size + 2, 25, true, packet_management);
		t = timeGetTime() - t;
		if (o < 0)
		{
			printf("error:%d\n", o);
			return 0;
		}

		size -= block_size;
		p += block_size;

		printf("\rremaining:%d, %d/%d\n", size, t, timeGetTime()-t2);
		t2 = timeGetTime();
		//Sleep(20);
	}

	// close on board file
	pkt->service = management_file_close;
	if (unicast_with_ack(nodeid, (uint8_t*)pkt, 1, 5, true, packet_management) < 0)
		return -2;

	delete data;
	free(pkt);

	upload_time = timeGetTime() - upload_time;
	printf("upload done, speed=%d byte/s\n", file_size * 1000 / upload_time);

	return 0;
}

int ZLink::play_file(int nodeid, const char *onboard_filename, int dt, int repeat)
{
	int64_t ts;
	if (get_station_time(&ts) < 0)
		return -1;

	management_info pkt;
	pkt.service = management_file_play;
	strcpy(pkt.file_play.filename, onboard_filename);
	pkt.file_play.ref_timestamp = ts + dt;

	if (nodeid != NODE_BROADCAST)
		return unicast_with_ack(nodeid, (uint8_t*)&pkt, sizeof(pkt.file_play), 5, true, packet_management);
	else
	{
		for(int i=0; i<repeat; i++)
			unicast_with_ack(NODE_BROADCAST, (uint8_t*)&pkt, sizeof(pkt.file_play), 5, true, packet_management);

		return 0;
	}
}

int ZLink::stop_file(int nodeid)
{
	management_info pkt;
	pkt.service = management_file_stop;

	return unicast_with_ack(nodeid, (uint8_t*)&pkt, 1, 5, true, packet_management);
}
int ZLink::hash_file(int nodeid, const char *onboard_filename)
{
	management_info pkt;
	pkt.service = management_file_hash_request;
	strcpy(pkt.file_hash_request.filename, onboard_filename);

	return unicast_with_ack(nodeid, (uint8_t*)&pkt, sizeof(pkt.file_hash_request), 5, true, packet_management);
}

int ZLink::decompress_file(int nodeid, const char *onboard_in_filename, const char *onboard_out_filename)
{
	management_info pkt;
	pkt.service = management_file_decompress_request;
	strcpy(pkt.file_decompress_request.in_filename, onboard_in_filename);
	strcpy(pkt.file_decompress_request.out_filename, onboard_out_filename);

	return unicast_with_ack(nodeid, (uint8_t*)&pkt, sizeof(pkt.file_decompress_request), 5, true, packet_management);
}

int ZLink::change_nodeid(int oldid, int newid)
{
	char tmp[1024];
	management_info *pkt = (management_info *)tmp;
	pkt->service = management_change_nodeid;
	pkt->change_nodeid.old_nodeid = oldid;
	pkt->change_nodeid.new_nodeid = newid;
	if (unicast_with_ack(oldid, (uint8_t*)pkt, sizeof(pkt->change_nodeid), 5, true, packet_management) < 0)
		return -1;

	return 0;
}

int ZLink::change_node_key(int nodeid, const uint8_t *aes_key)
{
	change_aes_key_info p;
	p.service = management_change_aes_key;
	memcpy(p.new_key, aes_key, 32);
	if (unicast_with_ack(nodeid, (uint8_t*)&p, sizeof(p), 5, true, packet_management) < 0)
		return -1;

	return 0;
}

int ZLink::set_node_airrate(int nodeid, uint8_t airrate)
{
	management_air_rate_info a = { management_air_rate, airrate };

	return unicast_with_ack(nodeid, (uint8_t*)&a, sizeof(a), 5, true, packet_management);
}

int ZLink::set_node_freq(int nodeid, float upfreq, float downfreq)
{
	change_freq_info a = { management_change_freq, upfreq*1e6,  downfreq*1e6};

	return unicast_with_ack(nodeid, (uint8_t*)&a, sizeof(a), 5, true, packet_management);
}

int ZLink::node_rgb(int nodeid, uint8_t R, uint8_t G, uint8_t B)
{
	char guard2[200];
	management_info *info = (management_info *)guard2;
	for (int i = 0; i < 140; i++)
		guard2[i] = i;
	info->service = management_RGB;
	info->RGB.R = R;
	info->RGB.G = G;
	info->RGB.B = B;
	return unicast_with_ack(nodeid, (uint8_t*)info, 4, 5, true, packet_management);
}

int ZLink::get_station_time(int64_t *time)
{
	int size = 8;
	int o = RPC(cmd_network_get_time, NULL, 0, (uint8_t*)time, &size);

	if (size != 8)
		return error_unknown;

	return o;
}
int ZLink::set_station_time(int64_t time)
{
	return RPC(cmd_network_set_time, (uint8_t*)&time, 8, 0, 0);
}

int ZLink::start_probing(int interval)
{
	return RPC(cmd_probing_start, (uint8_t*)&interval, 4, 0, 0);

}
int ZLink::stop_probing()
{
	return RPC(cmd_probing_stop, 0, 0, 0, 0);
}

int ZLink::set_airrate(uint8_t airrate)
{
	return RPC(cmd_network_set_rate, &airrate, 1, 0, 0);
}

int ZLink::bootloader()
{
	return RPC(cmd_debug_reboot_to_bootloader, NULL, 0, 0, 0);
}

#endif
