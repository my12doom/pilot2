#include "device_bulk.h"
#include <Windows.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "libusb.h"
#include "Protocol/crc32.h"
#pragma comment(lib, "libusb-1.0.lib");
#pragma comment(lib, "winmm.lib")

extern "C" 
{
#include <utils/minilzo.h>
}

int vendor_id = 0x111b;
int product_id = 0x1234;
static libusb_context* g_libusb_context = NULL;
static int (*rx)(void *buf, int len, int type) = NULL;
HANDLE usb_thread;
HANDLE rx_thread;
HANDLE usb_event;
bool working = true;
int16_t rx_data[65536];
int rx_data_size = 0;

DWORD WINAPI rx_worker(LPVOID p)
{
	while(working)
	{
		WaitForSingleObject(usb_event, INFINITE);
		if (rx && rx_data_size)
			rx(rx_data, rx_data_size, 1);
	}
	return 0;
}
#define HEAP_ALLOC(var,size) \
	lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]

static HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);

int erase_test(libusb_device_handle* dev_handle)
{
	static bool once = false;
	if (once)
		return 0;

	FILE * f = fopen("C:\\Users\\my12doom\\Desktop\\LED_CSG324\\led_test.bin", "rb");
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
	pkt.address = 0x08010008;
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
	pkt.address = 0x08010000;
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


DWORD WINAPI usb_worker(LPVOID p)
{
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

	libusb_init(&g_libusb_context);
start:
	libusb_device **devs;
	int cnt = libusb_get_device_list(NULL, &devs);

	libusb_device *dev;
	int i = 0;
	uint8_t path[8]; 
	while ((dev = devs[i++]) != NULL)
	{
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "failed to get device descriptor");
			return -1;
		}

		struct libusb_device_handle *dev_handle = NULL;
		libusb_open(dev, &dev_handle);
		if (dev_handle)
		{
			unsigned char manufacturer[200] = {0};
			unsigned char product[200] = {0};
			libusb_get_string_descriptor_ascii(dev_handle, desc.iManufacturer, manufacturer, 200);
			libusb_get_string_descriptor_ascii(dev_handle, desc.iProduct, product, 200);
			printf("%s %s : ", manufacturer, product);
			libusb_close(dev_handle);


			struct libusb_config_descriptor *config_descriptor;
			libusb_get_active_config_descriptor(dev, &config_descriptor);

			for(int j=0; j<config_descriptor->bNumInterfaces; j++)
			{
				const struct libusb_interface inf = config_descriptor->interface[j];

				printf("interface %d has %d endpoints\n", j, inf.altsetting[0].bNumEndpoints);
			}
		}
		else
		{
			printf("(Unkown) : ");
		}



		printf("%04x:%04x (bus %d, device %d)",
			desc.idVendor, desc.idProduct,
			libusb_get_bus_number(dev), libusb_get_device_address(dev));

		r = libusb_get_port_numbers(dev, path, sizeof(path));
		if (r > 0) {
			printf(" path: %d", path[0]);
			for (int j = 1; j < r; j++)
				printf(".%d", path[j]);
		}
		printf("\n\n");
	}


	libusb_device_handle* dev_handle = NULL;

	while(!dev_handle)
	{
		Sleep(100);
		dev_handle = libusb_open_device_with_vid_pid(g_libusb_context, vendor_id, product_id);

		if (!working)
			return 0;
	}

	int rslt = libusb_kernel_driver_active(dev_handle, 1);
	if(rslt != 0)
		libusb_detach_kernel_driver(dev_handle, 1);

	//int b = libusb_kernel_driver_active(dev_handle, 0);
	int config2;
	libusb_get_configuration(dev_handle,&config2);
	if(config2 != 1)
		libusb_set_configuration(dev_handle, 1);


	int r = libusb_claim_interface(dev_handle, 1);

	unsigned char data[65536] = {0};
	int actual_length = 64;
	int actual_length2 = 64;

	int c = 0;
	uint32_t l = timeGetTime();
	static uint32_t l2 = timeGetTime();

	uint8_t bulk_cmd[65536] = {0xE8};

	bulk_cmd[0] = 0x83;
	//int result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, bulk_cmd, 1024, &actual_length, 1000);

	Sleep(100);

	erase_test(dev_handle);

	while(1)
	{	
		int result;// = libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, bulk_cmd, 512, &actual_length, 0);
		result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_IN, data, 65536, &actual_length, 0);
//  		libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, data, actual_length, &actual_length2, 16);
// 		libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, data, actual_length, &actual_length2, 16);

		int dt = timeGetTime() - l2;
		l2 = timeGetTime();


		if (!working)
			break;

		if (result == -7)
			continue;

		if (result < 0)
			break;

		c += actual_length;

		int16_t *data16 = (int16_t*)data;

		int interval = timeGetTime() - l;
		if ( interval >= 1000)
		{
			l = timeGetTime();
			printf("%d bytes/s\n", (int64_t)c*interval/1000);
			c = 0;
		}

		int OSR = 1;
		int divide = 1;
		if (OSR >1 )
		for(int i=0; i<actual_length/2; i+=2*OSR)
		{
			int o[2] = {0};

			for(int j=0; j<OSR; j++)
			{
				o[0] += (data16[i+j*2])<<2;
				o[1] += (data16[i+j*2+1])<<2;
			}
			data16[i/OSR] = o[0]/divide + 400;
			data16[i/OSR+1] = -o[1]/divide + 500;
		}

		if (0)
		{
			static int16_t last;
			rx_data[0] = last;
			last = rx_data[actual_length/2/OSR-2];
			memcpy(rx_data+1, data16, actual_length/OSR-2);
			rx_data_size = actual_length/2/OSR;
		}
		else
		{
			memcpy(rx_data, data16, actual_length/OSR);
			rx_data_size = actual_length/2/OSR;
		}

		SetEvent(usb_event);
	}

	libusb_release_interface(dev_handle, 0);
	libusb_close(dev_handle);

	if (working)
		goto start;

	return 0;
}

int device_bulk_init(int (*rx)(void *buf, int len, int type))
{
	::rx = rx;

	usb_thread = CreateThread(NULL, NULL, usb_worker, NULL, NULL, NULL);
	rx_thread = CreateThread(NULL, NULL, rx_worker, NULL, NULL, NULL);
	usb_event = CreateEvent(NULL, FALSE, FALSE, NULL);
	return 0;
}

int device_bulk_exit()
{
	working = false;
	WaitForSingleObject(usb_thread, INFINITE);

	return 0;
}
