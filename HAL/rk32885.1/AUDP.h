#pragma once

#include <stdint.h>
#include <vector>
#include <HAL/Interface/IBlockDevice.h>
#include <pthread.h>

// net
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

namespace androidUAV
{
	// class for WIN32 UDP block device.
	typedef struct _packet
	{
		int size;
		uint8_t data[4096];
	} packet;


	class AUDP_RX : public HAL::IBlockDevice
	{
	public:
		AUDP_RX(const char*remote = INADDR_ANY, int port = 0xbbb);
		~AUDP_RX();

		int set_source(const char*remote, int port);

		// write a block
		// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
		// returns num bytes written, negative values for error.
		virtual int write(const void *buf, int block_size);

		// read a block from rx queue, remove block from the queue if remove == true.
		// returns num bytes read, negative values for error.
		virtual int read(void *buf, int max_block_size, bool remove = true);

		// query num available blocks in rx queue, negative values for error.
		virtual int available();

	protected:
		int socket_descriptor;
		struct sockaddr_in address;
		bool auto_redirection;
		bool worker_run;
		pthread_t worker_thread;
		pthread_mutex_t cs;
		std::vector<packet> packets;

		int clearup();

		void* worker();
		static void * worker_entry(void *p){return ((AUDP_RX*)p)->worker();}
	};

	class AUDP_TX : public HAL::IBlockDevice
	{
	public:
		AUDP_TX(const char*remote, int port);
		~AUDP_TX();

		int set_destination(const char*remote, int port);

		// write a block
		// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
		// returns num bytes written, negative values for error.
		virtual int write(const void *buf, int block_size);

		virtual int read(void *buf, int max_block_size, bool remove = true);

		// query num available blocks in rx queue, negative values for error.
		virtual int available();

	protected:
		int socket_descriptor;
		struct sockaddr_in address;

		int clearup();
	};
}
