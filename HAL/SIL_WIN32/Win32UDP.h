#pragma once

#include <stdint.h>
#include <vector>
#include <winsock.h>
#include <Windows.h>
#include <HAL/Interface/IBlockDevice.h>

namespace SIL_WIN32
{
	// class for WIN32 UDP block device.
	typedef struct _packet
	{
		int size;
		uint8_t data[4096];
	} packet;


	class Win32UDP_RX : public HAL::IBlockDevice
	{
	public:
		Win32UDP_RX(const char*remote = INADDR_ANY, int port = 0xbbb);
		~Win32UDP_RX();

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
		HANDLE worker_thread;
		CRITICAL_SECTION cs;
		std::vector<packet> packets;

		int clearup();

		DWORD worker();
		static DWORD WINAPI worker_entry(LPVOID p){return ((Win32UDP_RX*)p)->worker();}
	};

	class Win32UDP_TX : public HAL::IBlockDevice
	{
	public:
		Win32UDP_TX(const char*remote, int port);
		~Win32UDP_TX();

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
