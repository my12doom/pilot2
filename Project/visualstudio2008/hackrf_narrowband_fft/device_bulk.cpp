#include "device_bulk.h"
#include <Windows.h>
#include <stdint.h>
#include <stdio.h>
#include "libusb.h"
#pragma comment(lib, "libusb-1.0.lib");
#pragma comment(lib, "winmm.lib")

int vendor_id = 0x111b;
int product_id = 0x1234;
static libusb_context* g_libusb_context = NULL;
static int (*rx)(void *buf, int len, int type) = NULL;
HANDLE thread;
bool working = true;

DWORD WINAPI worker(LPVOID p)
{
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

	int c = 0;
	int l = GetTickCount();

	uint8_t bulk_cmd[1] = {0xE8};

	int result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, bulk_cmd, 1, &actual_length, 1000);

	Sleep(100);
	bulk_cmd[0] = 0xE2;
	result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_OUT, bulk_cmd, 1, &actual_length, 1000);

	while(1)
	{	
		int result= libusb_bulk_transfer(dev_handle, 1 | LIBUSB_ENDPOINT_IN, data, 32768, &actual_length, 16);

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

		for(int i=0; i<actual_length/2; i++)
			data16[i] = (data16[i]-2048)*16;

		if (rx)
			rx(data16, actual_length/2, 1);
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

	thread = CreateThread(NULL, NULL, worker, NULL, NULL, NULL);
	return 0;
}

int device_bulk_exit()
{
	working = false;
	WaitForSingleObject(thread, INFINITE);

	return 0;
}
