#include "Win32UDP.h"

#pragma comment(lib,"ws2_32.lib")

using namespace std;
using namespace HAL;

namespace SIL_WIN32
{

Win32UDP_RX::Win32UDP_RX(const char*remote /*= INADDR_ANY*/, int port /*= 0xbbb*/)
{
	InitializeCriticalSection(&cs);
	worker_thread = INVALID_HANDLE_VALUE;
	socket_descriptor = -1;

	set_source(remote, port);
}

Win32UDP_RX::~Win32UDP_RX()
{
	clearup();
	DeleteCriticalSection(&cs);
}

int Win32UDP_RX::clearup()
{
	// close socket and signal the worker thread to exit
	worker_run = false;
	if (socket_descriptor >= 0)
		closesocket(socket_descriptor);
	socket_descriptor = -1;
	
	// wait for thread to exit
	if (INVALID_HANDLE_VALUE != worker_thread)
		WaitForSingleObject(worker_thread, INFINITE);
	worker_thread = INVALID_HANDLE_VALUE;

	return 0;
}

int Win32UDP_RX::set_source(const char*remote, int port)
{
	static bool startup = true;
	if (startup)
	{
		startup = false;

		WSADATA wsaData;
		if (WSAStartup(MAKEWORD(1, 1), &wsaData) != 0) 
			return -1;
	}

	clearup();
	socket_descriptor = socket(AF_INET, SOCK_DGRAM, 0);  
	if (socket_descriptor <0) 
	{
		printf("error opening udp socket");
		return -1;
	}

	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl(INADDR_ANY);
	address.sin_port = htons(port);

	bind(socket_descriptor, (struct sockaddr *)&address, sizeof(address));

	// create worker thread if not running
	if (worker_thread == INVALID_HANDLE_VALUE)
	{
		worker_run = true;
		worker_thread = CreateThread(NULL, NULL, worker_entry, this, NULL, NULL);
	}

	return 0;
}

// write a block
// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
// returns num bytes written, negative values for error.
int Win32UDP_RX::write(const void *buf, int block_size)
{
	return error_unsupported;
}

// read a block from rx queue, remove block from the queue if remove == true.
// returns num bytes read, negative values for error.
int Win32UDP_RX::read(void *buf, int max_block_size, bool remove /*= true*/)
{
	EnterCriticalSection(&cs);

	if (packets.size() == 0)
	{
		LeaveCriticalSection(&cs);
		return error_no_more_blocks;
	}

	packet p = packets[0];
	if (remove)
		packets.erase(packets.begin());
	int size = min(max_block_size, p.size);
	memcpy(buf, p.data, size);
	LeaveCriticalSection(&cs);

	return size;
}

// query num available blocks in rx queue, negative values for error.
int Win32UDP_RX::available()
{
	EnterCriticalSection(&cs);

	int count = packets.size();

	LeaveCriticalSection(&cs);

	return count;
}

DWORD Win32UDP_RX::worker()
{
	while(worker_run)
	{
		packet p;

		address.sin_addr.s_addr = htonl(INADDR_ANY);
		int size_of_address = sizeof(address);
		p.size = recvfrom(socket_descriptor, (char*)p.data, sizeof(p.data), 0, (struct sockaddr *)&address, &size_of_address);

		EnterCriticalSection(&cs);

		if (p.size > 0)
			packets.push_back(p);

		LeaveCriticalSection(&cs);
	}

	return 0;
}



Win32UDP_TX::Win32UDP_TX(const char*remote, int port)
{
	socket_descriptor = -1;

	set_destination(remote, port);
}

Win32UDP_TX::~Win32UDP_TX()
{
	clearup();
}

int Win32UDP_TX::set_destination(const char*remote, int port)
{
	clearup();

	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr(remote);
	address.sin_port = htons(port);

	socket_descriptor = socket(AF_INET, SOCK_DGRAM, 0);  

	return 0;
}

int Win32UDP_TX::clearup()
{
	if (socket_descriptor > 0)
		closesocket(socket_descriptor);
	socket_descriptor = -1;

	return 0;
}

// write a block
// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
// returns num bytes written, negative values for error.
int Win32UDP_TX::write(const void *buf, int block_size)
{
	int o = sendto(socket_descriptor, (char*)buf, block_size, 0, (struct sockaddr *)&address, sizeof(address));
	return o<0 ? o : block_size;
}

int Win32UDP_TX::read(void *buf, int max_block_size, bool remove/* = true*/)
{
	return error_unsupported;
}

// query num available blocks in rx queue, negative values for error.
int Win32UDP_TX::available()
{
	return error_unsupported;
}


}
