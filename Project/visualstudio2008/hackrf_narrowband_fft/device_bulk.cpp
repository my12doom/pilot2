#include "device_bulk.h"
#include <Windows.h>
#include <stdio.h>
#include <assert.h>
#include <F4SDR/F4SDR.h>
#include "Protocol/crc32.h"
#pragma comment(lib, "libusb-1.0.lib");
#pragma comment(lib, "winmm.lib")

extern "C" 
{
#include <utils/minilzo.h>
}

using namespace F4SDR;


#define transfer_size (256*1024)
#define transfer_count 4
// #define use_transfers

struct libusb_transfer* transfers[transfer_count];


#define HEAP_ALLOC(var,size) \
	lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]

static HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);



int erase_test(libusb_device_handle* dev_handle)
{
	return 0;

	uint8_t d[256];

	int rr = libusb_control_transfer(
		dev_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		LIBUSB_REQUEST_GET_DESCRIPTOR,
		(3<<8) | 1,
		0,
		d,
		sizeof(d),
		0
		);

	static bool once = false;
	if (once)
		return 0;

	FILE * f = fopen("C:\\Users\\my12doom\\Desktop\\F4SA\\out\\adc_clk.bin", "rb");
	fseek(f, 0, SEEK_END);
	int data_size = ftell(f);
	fseek(f, 0, SEEK_SET);
	uint8_t *data = new uint8_t[data_size];
	fread(data, 1, data_size, f);
	fclose(f);
	bool lzo = true;

	if (lzo)
	{
		uint8_t *compressed = new uint8_t[data_size*2];
		lzo_uint compressed_len = 0;
		lzo_init();
		int r = lzo1x_1_compress(data, data_size, compressed, &compressed_len, wrkmem);

		if (r>=0 && compressed_len < data_size)
		{
			memcpy(data, compressed, compressed_len);
			data_size = compressed_len;
			delete compressed;
		}
		else
		{
			lzo = false;
			delete compressed;
		}
	}

	#pragma pack(push, 1)
	struct
	{
		uint8_t cmd;
		uint32_t address;
		int size;
		uint8_t data[512-12];
	} pkt;
	#pragma pack(pop)

	// erase
	int sizeo = sizeof(pkt);
	pkt.cmd = 0x81;
	int actual_length;
	int result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, (unsigned char*)&pkt, 512, &actual_length, 1000);

	printf("erasing...");
	//Sleep(10000);
	pkt.cmd = 0x84;
	while(libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, (unsigned char*)&pkt, 512, &actual_length, 1000) < 0)
		Sleep(10);
	printf("%s\n", "OK");

	// write
	pkt.cmd = 0x82;
	pkt.address = 0x08020008;
	int left = data_size;
	while(left>=0)
	{
		pkt.size = sizeof(pkt.data);
		memcpy(pkt.data, data+data_size-left, min(left, sizeof(pkt.data)));

		result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, (unsigned char*)&pkt, 512, &actual_length, 1000);

		printf("\r%d left      ", left);

		if (result < 0)
		{
			printf("error\n");

			continue;
		}

		pkt.address += sizeof(pkt.data);
		left -= sizeof(pkt.data);
	}

	// size & checksum
	pkt.cmd = 0x82;
	pkt.address = 0x08020000;
	pkt.size = 8;
	int32_t file_size_lzo = lzo ? (data_size | 0x80000000) : data_size;
	uint32_t crc = crc32(0, data, data_size);	
	memcpy(pkt.data, &file_size_lzo, 4);
	memcpy(pkt.data+4, &crc, 4);
	result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, (unsigned char*)&pkt, 1024, &actual_length, 1000);

	// reset
	pkt.cmd = 0x83;
	result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, (unsigned char*)&pkt, 1024, &actual_length, 1000);

	once = true;
	delete [] data;

	return 0;
}

int erase_test2(libusb_device_handle* dev_handle)
{

 	return 0;

	static bool once = false;
	if (once)
		return 0;

// 	FILE * f = fopen("C:\\Users\\mo\\Desktop\\repo\\F4SA\\out\\adc_clk.bin", "rb");
	FILE * f = fopen("C:\\Users\\mo\\Desktop\\repo\\VNA1Pro\\out\\VNA1pro.bin", "rb");
// 	FILE * f = fopen("Y:\\folder\\fpga_proj\\F4SA\\out\\adc_clk.bin", "rb");
	fseek(f, 0, SEEK_END);
	int data_size = ftell(f);
	fseek(f, 0, SEEK_SET);
	uint8_t *data = new uint8_t[data_size];
	fread(data, 1, data_size, f);
	fclose(f);
	bool lzo = true;

	if (lzo)
	{
		uint8_t *compressed = new uint8_t[data_size*2];
		lzo_uint compressed_len = 0;
		lzo_init();
		int r = lzo1x_1_compress(data, data_size, compressed, &compressed_len, wrkmem);

		if (r>=0 && compressed_len < data_size)
		{
			memcpy(data, compressed, compressed_len);
			data_size = compressed_len;
			delete compressed;
		}
		else
		{
			lzo = false;
			delete compressed;
		}
	}

	// erase

	uint8_t tmp[64];

	printf("erasing...");
	*(uint32_t*)tmp = 0xDEADBEAF;
	int result = libusb_control_transfer(dev_handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		erase_fpga_rom, 0, 0, tmp, 4, 1000);
	if (result < 0)
	{
		printf("failed\n");
		return -1;
	}

	// wait for erasure to finish
	while(libusb_control_transfer(dev_handle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		ping, 0, 0, tmp, 4, 1000) < 0)
	{
		Sleep(10);
	}
	printf("%s\n", "OK");

	// write
	uint32_t address = 0x08020008;
	int left = data_size;
	int MTU = sizeof(tmp);
	
	while(left>0)
	{
		int block_size = min(left, MTU);
		memcpy(tmp, data+data_size-left, block_size);

		result= libusb_control_transfer(dev_handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			write_fpga_rom, address>>16, address, tmp, block_size, 1000);
		printf("\r%d left      ", left);

		if (result < 0)
		{
			printf("error\n");

			return -1;
		}

		address += block_size;
		left -= block_size;
	}

	// size & checksum
	address = 0x08020000;
	int32_t file_size_lzo = lzo ? (data_size | 0x80000000) : data_size;
	uint32_t crc = crc32(0, data, data_size);	
	memcpy(tmp, &file_size_lzo, 4);
	memcpy(tmp+4, &crc, 4);

	result = libusb_control_transfer(dev_handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		write_fpga_rom, address>>16, address, tmp, 8, 1000);

	// reset
	result= libusb_control_transfer(dev_handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		reset_mcu, 0, 0, tmp, 4, 1000);

	once = true;
	delete [] data;

	return result;
}

namespace NBFFT
{
void bulk_device::libusb_transfer_callback(struct libusb_transfer* usb_transfer)
{
	if(usb_transfer->status == LIBUSB_TRANSFER_COMPLETED)
	{

		handle_data(usb_transfer->buffer, usb_transfer->actual_length);

		libusb_submit_transfer(usb_transfer);
		/*
		hackrf_transfer transfer = {
			.device = device,
			.buffer = usb_transfer->buffer,
			.buffer_length = usb_transfer->length,
			.valid_length = usb_transfer->actual_length,
			.rx_ctx = device->rx_ctx,
			.tx_ctx = device->tx_ctx
		};
		*/


		//usb_transfer->
	}
	else
	{
		// error handling here
	}
}
int bulk_device::handle_data(uint8_t *data, int actual_length)
{
	int16_t *data16 = (int16_t*)data;

	static int l = timeGetTime();
	static int c = 0;
	c += actual_length;
	int interval = timeGetTime() - l;
	if ( interval >= 1000)
	{
		l = timeGetTime();
		printf("%lld bytes/s(%.3fMsps)\n", (int64_t)c*1000/interval, (double)c/interval/1000/4);
		c = 0;
	}

	int divide = 1;
	uint16_t *udata16 = (uint16_t*)data16;

	memcpy(rx_data, data16, actual_length);
	rx_data_size = actual_length/2;

	SetEvent(usb_event);
	return 0;
}

DWORD bulk_device::usb_worker()
{
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

	libusb_context* ctx = NULL;

	libusb_init(&ctx);
	usb_ctx = ctx;
	libusb_set_debug(ctx, 3);

start:


// 	libusb_device_handle* dev_handle;
	dev_handle = NULL;

	while(!dev_handle)
	{
		Sleep(100);
		dev_handle = libusb_open_device_with_vid_pid(ctx, F4SDR_VENDOR_ID, F4SDR_PRODUCT_ID);

		if (!working)
			return 0;
	}

	Sleep(100);
	int rslt = libusb_kernel_driver_active(dev_handle, 1);
	if(rslt != 0)
		rslt = libusb_detach_kernel_driver(dev_handle, 1);

	rslt = libusb_kernel_driver_active(dev_handle, 0);
	if(rslt != 0)
		rslt = libusb_detach_kernel_driver(dev_handle, 0);

	//int b = libusb_kernel_driver_active(dev_handle, 0);
	int config2;
	rslt = libusb_get_configuration(dev_handle,&config2);
	if(config2 != 1)
		rslt = libusb_set_configuration(dev_handle, 1);


	int r = libusb_claim_interface(dev_handle, 0);
	r = libusb_claim_interface(dev_handle, 1);

	unsigned char data[transfer_size] = {0};
	int actual_length = 64;
	int actual_length2 = 64;

	static uint32_t l2 = timeGetTime();

	uint8_t bulk_cmd[transfer_size] = {0xE8};

	bulk_cmd[0] = 0x83;
	//int result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, bulk_cmd, 1024, &actual_length, 1000);

	Sleep(100);

	erase_test2(dev_handle);

	// create and prepare transfers
#ifdef use_transfers
	uint8_t *buf = (uint8_t*)malloc(1024*1024);;
	for(int i=0; i<transfer_count; i++)
	{
		transfers[i] = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfers[i], dev_handle, 1|LIBUSB_ENDPOINT_IN, &buf[i * transfer_size], transfer_size, libusb_transfer_callback_entry, this, 0);
		int result = libusb_submit_transfer(transfers[i]);
	}
#endif

	while(1)
	{

#ifndef use_transfers
		int result = 0;
		EnterCriticalSection(&cs_usb);
 		result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_IN, data, transfer_size, &actual_length, 0);
		//Sleep(10);
		LeaveCriticalSection(&cs_usb);

		int dt = timeGetTime() - l2;
		l2 = timeGetTime();


		if (!working)
			break;

		if (result == -7)
			continue;

		if (result < 0)
			break;


		handle_data(data, actual_length);

#else

		if (!working)
			break;
		struct timeval timeout = { 0, 500000 };
		int error = libusb_handle_events_timeout(ctx, &timeout);
		if (error <0)
			break;

#endif
	}

#ifdef use_transfers
	for(int i=0; i<transfer_count; i++)
 		libusb_cancel_transfer(transfers[i]);
#endif

	r = libusb_release_interface(dev_handle, 0);
	libusb_close(dev_handle);

#ifdef use_transfers
	Sleep(100);
	for(int i=0; i<transfer_count; i++)
		libusb_free_transfer(transfers[i]);
	free(buf);
#endif


	if (working)
		goto start;

	return 0;

}

int bulk_device::init(int (*rx)(void *buf, int len))
{
	this->cb = rx;
	working = true;
	rx_data_size = 0;

	usb_thread = CreateThread(NULL, NULL, usb_worker_entry, this, NULL, NULL);
	rx_thread = CreateThread(NULL, NULL, rx_worker_entry, this, NULL, NULL);
	usb_event = CreateEvent(NULL, FALSE, FALSE, NULL);
	return 0;
}

int bulk_device::destroy()
{
	working = false;
	SetEvent(usb_event);
	WaitForSingleObject(usb_thread, INFINITE);
	WaitForSingleObject(rx_thread, INFINITE);
	if (usb_event != INVALID_HANDLE_VALUE)
		CloseHandle(usb_event);
	usb_event = INVALID_HANDLE_VALUE;
	return 0;
}


DWORD bulk_device::rx_worker()
{
	while(working)
	{
		WaitForSingleObject(usb_event, INFINITE);
		if (working && cb && rx_data_size)
			cb(rx_data, rx_data_size);
	}
	return 0;
}

int bulk_device::tune(int64_t hz)
{
	EnterCriticalSection(&cs_usb);
	if (!dev_handle)
	{
		LeaveCriticalSection(&cs_usb);
		return -1;
	}
	int ret = libusb_control_transfer(dev_handle,LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		::tune, 0, 0,	(uint8_t*)&hz, 8,10 );
	LeaveCriticalSection(&cs_usb);

	return ret;
}
int bulk_device::config()
{
// 	uint8_t tmp[24];
// 	int res = libusb_control_transfer(
// 		dev_handle,
// 		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
// 		0x85, 1, 2,	tmp, 4,0 );
// 
// 	int retval = 0x12345678;

	int dummy = 0;

	EnterCriticalSection(&cs_usb);
	if (!dev_handle)
	{
		LeaveCriticalSection(&cs_usb);
		return -1;
	}
	int result = libusb_control_transfer(
		dev_handle,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		reset_mcu, 1, 2,	(uint8_t*)&dummy, 0,0 );

	LeaveCriticalSection(&cs_usb);
	return result;
}
int bulk_device::set_gains(uint8_t *gains)
{
	EnterCriticalSection(&cs_usb);
	if (!dev_handle)
	{
		LeaveCriticalSection(&cs_usb);
		return -1;
	}
	int result = libusb_control_transfer(
		dev_handle,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		set_gain, 1, 2,	gains, get_gains_count(),0 );

	LeaveCriticalSection(&cs_usb);
	return result;

}


int bulk_device::get_gains(uint8_t *gains)
{
	EnterCriticalSection(&cs_usb);
	if (!dev_handle)
	{
		LeaveCriticalSection(&cs_usb);
		return -1;
	}
	int result = libusb_control_transfer(
		dev_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		get_gain, 1, 2,	gains, get_gains_count(),0 );

	bool iflna = true;
	uint8_t ifpath = IF_900;
	uint8_t rfpath = FE_LNA;

	uint8_t path = (rfpath << 5) | (ifpath << 1) | (iflna?0x10:0);

	int n = libusb_control_transfer(dev_handle,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		F4SDR::set_path, 0, 0, &path, 1, 0);

	LeaveCriticalSection(&cs_usb);
	return result;

}

}
