#pragma once

#include <stdint.h>
#include <list>
#include <HAL/Interface/IBlockDevice.h>
#include <pthread.h>
#include <pcap/pcap.h>

namespace androidUAV
{
	// class for UDP block device.
	const int MAX_PACKET_LENGTH = 4096;
	typedef struct _packet
	{
		int size;
		uint8_t data[MAX_PACKET_LENGTH];
	} packet;


	class APCAP_RX : public HAL::IBlockDevice
	{
	public:
		APCAP_RX(const char*interface = "wlan0", int port = 0);
		~APCAP_RX();

		// write a block
		// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
		// returns num bytes written, negative values for error.
		virtual int write(const void *buf, int block_size);

		// read a block from rx queue, remove block from the queue if remove == true.
		// returns num bytes read, negative values for error.
		virtual int read(void *buf, int max_block_size, bool remove = true);

		// query num available blocks in rx queue, negative values for error.
		virtual int available();

		// return latest rssi in dbm.
		int get_latest_rssi(){return latest_rssi;}

		//
		// offset:0:center, 1:upper, 2:lower, bw: 0: 40M/20M, 1:10M, 2:5M
		int set_rf(int frequency, uint8_t offset, uint8_t bw);
		int set_rf2(int frequency, uint8_t offset, uint8_t bw, int8_t ant = -1);

		//
		int set_mcs_bw(int mcs, int bw);


	protected:
		uint8_t packet_transmit_buffer[MAX_PACKET_LENGTH];
		bool worker_run;
		pthread_t worker_thread;
		pthread_mutex_t cs;
		std::list<packet> packets;
		bool init_ok;
		pcap_t *ppcap;
		int n80211HeaderLength;
		int selectable_fd;
		int latest_rssi;

		int guard[1024];
		char ifname[100];
		int byte_counter;

		int clearup();

		void* worker();
		static void * worker_entry(void *p){return ((APCAP_RX*)p)->worker();}

	};

	typedef APCAP_RX APCAP_TX;
}
