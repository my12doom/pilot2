#include "device_fx3.h"
#include "FX3/CyAPI.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <string>

#pragma comment(lib, "setupapi.lib")
char fpga_bin_file[] = "C:\\Users\\mo\\Desktop\\repo\\yaDSO\\out\\yadso.bin";
char fx3_firmware[] = "C:\\Users\\mo\\Desktop\\repo\\yaDSO_FX3\\yaDSO.img";
// char fpga_bin_file[] = "vna1pro.bin";
// char fx3_firmware[] = "SDDC_FX3.img";

/* List of supported programming targets */
typedef enum {
	FW_TARGET_NONE = 0,	/* Invalid target					*/
	FW_TARGET_RAM,		/* Program firmware (hex) to RAM	*/
	FW_TARGET_I2C,		/* Program firmware (hex) to EEPROM	*/
	FW_TARGET_SPI		/* Program firmware (hex) to SPI	*/
} fx3_fw_tgt_p;

USHORT vendor_id = 0x04B4;
USHORT bootLoader_product_id = 0x00F3;
USHORT bootLoader2_product_id = 0x00F0;
USHORT fpga_pid = 0x00F1;

namespace NBFFT
{

	CCyFX3Device *open_device(int vid, int pid)
	{
		CCyFX3Device *m_usbDevice = new CCyFX3Device();
		if (m_usbDevice == NULL)
		{
			fprintf (stderr, "Error: Failed to create USB Device\n");
			return NULL;
		}

		for (int i = 0; i < m_usbDevice->DeviceCount(); i++)
		{
			if (m_usbDevice->Open((UCHAR)i))
			{
				if (m_usbDevice->VendorID == vid && m_usbDevice->ProductID == pid)
					return m_usbDevice;

				m_usbDevice->Close() ;
			}
		}

		delete m_usbDevice;

		return NULL;
	}

	int fx3_device::init(data_callback cb)
	{
		this->cb = cb;
		exiting = true;
		FX3_FWDWNLOAD_ERROR_CODE dwld_status = FAILED;
		char tgt_str[]  = "RAM";
		bool flag = false;
		bool status = false;
		int count = 0;
		int fileSz = 0;


		CCyFX3Device * RAM_Programmer = (CCyFX3Device *)open_device(vendor_id, bootLoader_product_id);
		if (!RAM_Programmer)
			RAM_Programmer = (CCyFX3Device *)open_device(vendor_id, bootLoader2_product_id);


		if (RAM_Programmer)
		{
			dwld_status = RAM_Programmer->DownloadFw(fx3_firmware, RAM);

			if(dwld_status == SUCCESS)
			{
				printf ("Info : Programming to RAM completed\n");
				Sleep(500);
			}
			else
			{
				printf ("Info : Programming to RAM failed:%d%s\n", dwld_status, INVALID_FILE == dwld_status ? "(firmware file not found)" : "");
			}

			RAM_Programmer->Close();
			delete RAM_Programmer;
		}

		exiting = false;
		thread = CreateThread(NULL, NULL, usb_worker_entry, this, NULL, NULL);

		return 0;
	}

	int fx3_device::destroy()
	{
		if (thread != INVALID_HANDLE_VALUE)
		{
			exiting = true;
			WaitForSingleObject(thread, INFINITE);
		}

		return 0;
	}
	DWORD fx3_device::usb_worker()
	{
		CCyFX3Device * device = open_device(vendor_id, fpga_pid);

		if (!device)
		{
			printf("no FX3 device found\n");
			return -1;
		}

		uint8_t tmp[256];
		bootloader_command = false;

		uint16_t value = 0;
		uint16_t index = 0;
		vendorCmdData vcmd = 
		{
			tmp, 0xB1, 	value | (index << 16), 4, true
		};

		printf("get fpga state...");
		if (!device->Ep0VendorCommand(vcmd))
		{
			printf("FAIL\n");
			return -1;
		}

		printf("OK, done=%d\n", tmp[0]);

// 	if (tmp[0] == 1)
// 	{
// 		printf("skip to speed test\n");
// 		goto speed_test;
// 	}

		printf("reading fpga file...");
		FILE * f = fopen(fpga_bin_file, "rb");

		if (!f)
		{
			printf("FAIL\n");
			return -3;
		}

		fseek(f, 0, SEEK_END);
		int filesize = ftell(f);
		fseek(f, 0, SEEK_SET);
		uint8_t *data = new uint8_t[filesize];
		fread(data, 1, filesize, f);
		fclose(f);
		printf("OK, %d bytes\n", filesize);

		memcpy(tmp+4, &filesize, 4);
		tmp[0] = tmp[4];
		tmp[1] = tmp[5];
		tmp[2] = tmp[6];
		tmp[3] = 0;//tmp[4];
		vcmd.opCode = 0xB2;
		vcmd.isRead = false;

		printf("programming fpga...");
		if (!device->Ep0VendorCommand(vcmd))
		{
			printf("FAIL\n");
			return -4;
		}

		int MTU = 1024;
		int left = filesize;
		uint8_t *p = data;

		while(left)
		{
			LONG size = left > MTU ? MTU : left;
			if (!device->BulkOutEndPt->XferData(p, size))
			{
				printf("FAIL2\n");
				return -5;
			}

			left -= size;
			p += size;
		}

		printf("OK\n");

loop:

		int xfer_size = 16384 * 16;
		int used_bytes = 0;
		int total_bytes = 0;
		int tick = GetTickCount();

		// prepare and commit xfers
		const int xfer_count = 16;
		OVERLAPPED* xfer_ov[xfer_count] = {0};
		PUCHAR xfer_contexts[xfer_count] = {0};
		uint8_t* ptr[xfer_count];
		for(int i=0; i<xfer_count; i++)
		{
			xfer_ov[i] = new OVERLAPPED;
			memset(xfer_ov[i], 0 , sizeof(OVERLAPPED));
			xfer_ov[i]->hEvent = CreateEvent(NULL, false, false, NULL);
			ptr[i] = new uint8_t[xfer_size];		
			xfer_contexts[i] = device->BulkInEndPt->BeginDataXfer(ptr[i], xfer_size, xfer_ov[i]);
		}

		int err_count = 0;
		while(!exiting)
		{
			// wait for first in queue to finish
			bool err = device->BulkInEndPt->WaitForXfer(xfer_ov[0], 100);
			if (!err)
			{
				device->BulkInEndPt->Abort();
				err = device->BulkInEndPt->WaitForXfer(xfer_ov[0], 100);
				err_count ++;
				if (err_count > 10)
					break;

			}

			LONG got = xfer_size;
			err = device->BulkInEndPt->FinishDataXfer(ptr[0], got, xfer_ov[0], xfer_contexts[0]);

			if (!err || got != xfer_size)
				err_count ++;

			// handle data here
			if (cb && got)
			{
				int16_t *p = (int16_t*)ptr[0];
				int count = xfer_size / 2;

				if (cb(p, xfer_size/2) == 0)
					used_bytes += got;
			}

			total_bytes += got;

			int l = GetTickCount();
			if (l - tick > 200)
			{
				printf("\rspeed:%d/%d Kbytes/s, %d error", used_bytes/(l-tick), total_bytes/(l-tick), err_count);
				tick = l;
				used_bytes = 0;
				total_bytes = 0;
			}

			// remove from queue
			uint8_t *ptr0 = ptr[0];
			OVERLAPPED *ov0 = xfer_ov[0];
			for(int i=1; i<xfer_count; i++)
			{
				xfer_ov[i-1] = xfer_ov[i];
				xfer_contexts[i-1] = xfer_contexts[i];
				ptr[i-1] = ptr[i];
			}

			// handle bootloader command
			if (bootloader_command)
			{
				bootloader_command = false;
				uint16_t value = 0;
				uint16_t index = 0;
				vendorCmdData vcmd = 
				{
					tmp, 0xC1, 	value | (index << 16), 4, true
				};

				printf("reset to bootloader...\n");
				if (!device->Ep0VendorCommand(vcmd))
				{
					printf("FAIL\n");
					return -1;
				}
			}

			// commit transfer and rejoin queue
			xfer_contexts[xfer_count-1] = device->BulkInEndPt->BeginDataXfer(ptr0, xfer_size, ov0);
			xfer_ov[xfer_count-1] = ov0;
			ptr[xfer_count-1] = ptr0;
		}

		// wait for everything to finish or timeout
		for(int i=0; i<xfer_count; i++)
		{
			for(int j=0; j<10; j++)
			{
				device->BulkInEndPt->Abort();
				if (WaitForSingleObject(xfer_ov[i]->hEvent, 100) == WAIT_OBJECT_0)
					break;
			}
		}


		for(int i=0; i<xfer_count; i++)
		{
			delete ptr[i];
			CloseHandle(xfer_ov[i]->hEvent);
			delete xfer_ov[i];
			// TODO: xfer_contexts leaked
		}

		device->Close();
		delete device;

		return 0;
	}

}
