#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <HAL/Interface/IUART.h>
#include <vector>
#include "ZLinkWireless.h"

// note: all structures are little endian (ARM/x86)
#pragma pack(push, 1)

#define ZLinkHeader 0xA385
#define HEADER_SIZE sizeof(ZLinkUSBFrame)
#define MAX_USB_DATA_SIZE 512

#define SEQUENCE_DOWNLINK 32768
#define SEQUENCE_INVALID 0xffff
#define SEQUENCE_MAX 0x8000

//
enum ZLinkUSBCmds;

typedef struct
{
	uint16_t header;		// big endian 0x85A3, little endian 0xA385
	uint16_t crc;			// crc16 of remaining data
	uint16_t size;			// size of body
	uint16_t sequence_id;
	uint8_t cmd;
	uint8_t flag;
	uint8_t data[];
} ZLinkUSBFrame;

typedef struct
{
	uint16_t nodeid;
	int8_t rssi;
	int8_t rssi_onboard;
	int64_t last_update;
} ZLinkNodeState;

enum ZLinkUSBFlags
{
	frame_flag_slow_rpc = 1<<0,
	frame_flag_END = 1<<7,
};

enum ZLinkUSBCmds
{
	cmd_nop = 0xff,
	cmd_ack = 0,
	cmd_nack = 1,

	cmd_network = 0x10,
	cmd_network_set_aes = 0x11,
	cmd_network_get_channels = 0x12,
	cmd_network_get_channel_capacity = 0x13,
	cmd_network_set_channels = 0x14,
	cmd_network_get_time = 0x19,
	cmd_network_set_time = 0x1A,
	cmd_network_get_freq = 0x1B,
	cmd_network_set_freq = 0x1C,
	cmd_network_set_rate = 0x1D,
	cmd_network_get_rate = 0x1E,
	cmd_probing_start = 0x15,
	cmd_probing_stop = 0x16,
	cmd_probing_get_list = 0x17,
	cmd_probing_get_state = 0x18,

	cmd_tx = 0x20,
	cmd_tx_raw = 0x21,
	cmd_tx_oneshot = 0x22,
	cmd_tx_withack = 0x23,

	cmd_rx = 0x31,

	cmd_node_set_aes = 0x41,
	cmd_node_get_id = 0x42,
	cmd_node_set_id = 0x43,

	cmd_debug_reset_rf = 0x51,
	cmd_debug_reboot_to_bootloader = 0x52,

};

enum ZLinkError
{
	error_OK = 0,
	error_unknown = -1,
	error_nack = -2,
	error_timeout = -3,
	error_result_timeout = -4,
	error_fail = -5,
};

void update_crc(ZLinkUSBFrame * frame);
bool check_crc(ZLinkUSBFrame *frame);
int search_frame(HAL::IUART *uart, ZLinkUSBFrame *frame);
ZLinkUSBFrame *new_frame(uint16_t data_size, uint16_t sequence_id, uint8_t cmd, uint8_t flag, const uint8_t *data);
ZLinkUSBFrame *clone_frame(ZLinkUSBFrame *frame);
void free_frame(ZLinkUSBFrame *p);


#ifdef _WIN32
#include <Windows.h>

typedef void (*RPC_callback)(ZLinkUSBFrame *frame, const void *p);
typedef struct
{
	RPC_callback cb;
	const void *p;
	uint16_t sequence_id;
	DWORD timeout;
	bool sync;
} sequence;

class ZLinkUSB
{
public:
	ZLinkUSB();
	~ZLinkUSB();

	int connect(HAL::IUART *uart);
	int disconnect();

	// synchronized RPC
	// use this ONLY FOR fast RPCs like configuration and udp-like TX, or don't care about performance
	int RPC(uint8_t cmd, const uint8_t *data, int data_count, uint8_t *out_data = NULL, int *out_data_count = NULL);

	// async RPC
	int RPC_async(uint8_t cmd, const uint8_t *data, int data_count, bool wait_ack = false, RPC_callback cb = NULL, const void *userdata = NULL);

	// interface timeout only, not wireless timeout
	int set_timeout(int milliseconds);

	int set_downlink_handler(RPC_callback cb);

	bool is_node();
	int node_set_id(uint16_t new_id);
	int node_get_id(uint16_t *id);
	int node_set_aes(const uint8_t *new_key);

	int set_freq(float uplink, float downlink);
	int get_freq(float *uplink, float *downlink);
protected:
	bool thread_exit;
	HANDLE h_rx_thread;
	DWORD rx_thread();
	static DWORD WINAPI rx_thread_entry(LPVOID p) { return ((ZLinkUSB*)p)->rx_thread();}
	HANDLE h_async_thread;
	DWORD async_thread();
	static DWORD WINAPI async_thread_entry(LPVOID p) { return ((ZLinkUSB*)p)->async_thread(); }


	uint16_t sequence_id;
	int timeout;
	HAL::IUART *uart;


	std::vector<ZLinkUSBFrame*> rx_frames;
	CRITICAL_SECTION cs;

	sequence on_going_sequences[65536];
	bool sequence_valid[65536];
	CRITICAL_SECTION cs2;

	std::vector<ZLinkUSBFrame*> async_frames;
	CRITICAL_SECTION cs3;

	void start_sequence(uint16_t sequence_id, bool sync, DWORD timeout, RPC_callback cb = NULL, const void *userdata = NULL);
	void end_sequence(uint16_t sequence_id);
	bool is_valid_sequence(uint16_t sequence_id, bool sync);
	sequence get_sequence(uint16_t sequence_id);

	RPC_callback downlink_cb;
};


class ZLink : public ZLinkUSB
{
public:
	ZLink()
	:ZLinkUSB()
	{}
	~ZLink() {}

	// TX/RX, node managements
	int tx_raw(const uint8_t *data, int size);
	int unicast_noack(int nodeid, const uint8_t *data, int size);
	int unicast_with_ack(int nodeid, const uint8_t *data, int size, int max_retry, bool blocking = false, uint8_t type = packet_payload);
	int broadcast(const uint8_t *data, int size);
	int unicast_with_ack(WirelessPacket *p, int max_retry, bool blocking = false);
	int reset_node(int nodeid);
	int change_nodeid(int oldid, int newid);
	int change_node_key(int nodeid, const uint8_t *aes_key);
	int set_node_airrate(int nodeid, uint8_t airrate);
	int set_node_freq(int nodeid, float upfreq, float downfreq);
	int node_rgb(int nodeid, uint8_t R, uint8_t G, uint8_t B);

	// node file ops
	int upload_file(int nodeid, const char *file, const char *onboard_filename);
	int play_file(int nodeid, const char *onboard_filename, int dt = 1000000, int repeat = 1);
	int stop_file(int nodeid);
	int hash_file(int nodeid, const char *onboard_filename);
	int decompress_file(int nodeid, const char *onboard_in_filename, const char *onboard_out_filename);

	// station managements
	int start_probing(int interval = 1000000);
	int stop_probing();
	int get_station_time(int64_t *time);
	int set_station_time(int64_t time);
	int set_airrate(uint8_t airrate);
	int bootloader();

//protected:
};

#endif

#pragma pack(pop)
