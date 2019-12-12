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

	protected:
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
		int byte_counter;

		int clearup();

		void* worker();
		static void * worker_entry(void *p){return ((APCAP_RX*)p)->worker();}

	};

	class APCAP_TX : public HAL::IBlockDevice
	{
	public:
		APCAP_TX(const char*interface, int port);
		~APCAP_TX();

		// write a block
		// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
		// returns num bytes written, negative values for error.
		virtual int write(const void *buf, int block_size);

		virtual int read(void *buf, int max_block_size, bool remove = true);

		// query num available blocks in rx queue, negative values for error.
		virtual int available();

		int set_mcs_bw(int mcs, int bw);
	protected:
		bool init_ok;
		pcap_t *ppcap;
		uint8_t packet_transmit_buffer[MAX_PACKET_LENGTH];
	};
}
