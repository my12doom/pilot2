#include "UartUbloxNMEAGPS.h"
#include <stdlib.h>
#include <stdio.h>
#include <HAL/Interface/ISystimer.h>

namespace sensors
{

UartUbloxNMEAGPS::UartUbloxNMEAGPS()
:UartNMEAGPS()
{
	current_baudrate = 0;
}

UartUbloxNMEAGPS::~UartUbloxNMEAGPS()
{

}

int UartUbloxNMEAGPS::enable_message(uint8_t cls, uint8_t id, bool enable/* = true*/, int timeout /* = 100000 */)
{
	char payload[] = {cls, id, 0,enable?1:0,0,0,0,0};
	send_ubx_packet(0x6, 0x1, payload, 8);
	uart->flush();

	// poll result
	int64_t timeout_tick = timeout>=0 ? (systimer->gettime() + timeout) : 0x7fffffffffffffff;
	ubx_packet *pkt;
	do
	{
		pkt = read_ubx_packet(timeout);

		if (pkt)
		{
			if (pkt->cls == 0x5)
				return (pkt->id == 1 && pkt->crc_ok) ? 0 : -2;
		}
		else
		{
			send_ubx_packet(0x6, 0x1, payload, 8);
			uart->flush();
		}
	}while(systimer->gettime() < timeout_tick);

	return -1;	
}

int UartUbloxNMEAGPS::trig_ubx_packet(int timeout)
{
	char payload[] = {0xf0, 0x00, 0, 1,0,0,0,0};

	// poll result
	int64_t timeout_tick = timeout>=0 ? (systimer->gettime() + timeout) : 0x7fffffffffffffff;
	ubx_packet *pkt;
	do
	{
		pkt = read_ubx_packet(timeout);

		if (pkt)
		{
			return 0;
		}
		else
		{
			send_ubx_packet(0x6, 0x1, payload, 8);
			uart->flush();
		}
	}while(systimer->gettime() < timeout_tick);

	return -1;	
}

int UartUbloxNMEAGPS::set_baudrate(int baudrate)
{
	if (detect_baudrate()<0)
		return -2;


	char payload[] = {0x1, 0x0, 0x0, 0x0, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	*(uint32_t*)(payload+8) = baudrate;

	int retry = 3;
	while(retry--)
	{
		send_ubx_packet(0x6, 0x0, payload, sizeof(payload));
		uart->flush();
		ubx_packet *pkt = NULL;
		do
		{
			pkt = read_ubx_packet(0);

			if (pkt && pkt->crc_ok && pkt->cls == 0x5 && pkt->id == 0x1 && pkt->payload_size >= 2)
				return detect_baudrate();
		}
		while(pkt);
	}

	return detect_baudrate();
}

int UartUbloxNMEAGPS::open_as(int baudrate)
{
	if (uart->flush() < 0)
		return -1;
	
	if (uart->set_baudrate(baudrate) < 0)
		return -1;

	current_baudrate = baudrate;

	// clear pending buffer
	char tmp[100];
	while(uart->available())
		if (uart->read(tmp, 100) < 0)
			return -2;

	return 0;
}

int UartUbloxNMEAGPS::detect_baudrate()
{
	int baudtable[] = {9600, 115200};

	// switch current baudrate to current baudrate
	for(int i=0; i<sizeof(baudtable)/sizeof(baudtable[0]); i++)
	{
		if (baudtable[i] == current_baudrate)
		{
			baudtable[i] = baudtable[0];
			baudtable[0] = current_baudrate;
		}

	}
	
	for(int i=0; i<sizeof(baudtable)/sizeof(baudtable[0]); i++)
	{
		if (open_as(baudtable[i]) < 0)
			break;
		
		if (trig_ubx_packet(2000000000/baudtable[i]) == 0)
			return baudtable[i];
	}	
	
	return -1;	
}

int UartUbloxNMEAGPS::send_ubx_packet(uint8_t cls, uint8_t id, void *payload, uint16_t payload_size)
{
	// 	uint8_t * data = new uint8_t[payload_size+8];
	uint8_t header[6];
	uint8_t crc[2] = {0};
	header[0] = 0xB5;
	header[1] = 0x62;
	header[2] = cls;
	header[3] = id;
	header[4] = payload_size & 0xff;
	header[5] = payload_size >> 8;

	for(int i=0; i<4; i++)
	{
		crc[0] = crc[0] + header[i+2];
		crc[1] = crc[1] + crc[0];
	}
	for(int i=0; i<payload_size; i++)
	{
		crc[0] = crc[0] + ((uint8_t*)payload)[i];
		crc[1] = crc[1] + crc[0];		
	}

	if (uart->write(header, 6) < 0)
		return -1;
	if (uart->write(payload, payload_size) < 0)
		return -1;
	if (uart->write(crc, 2) < 0)
		return -1;

	return 0;
}

ubx_packet* UartUbloxNMEAGPS::read_ubx_packet(int timeout)
{
	if (uart->available() < 2)
		return NULL;

	uint8_t data[2];
	uart->read(data+1, 1);

	int64_t timeout_tick = timeout>=0 ? (systimer->gettime() + timeout) : 0x7fffffffffffffff;

	do
	{
		data[0] = data[1];
		if (uart->read(data+1,1) != 1)
			continue;

		if (data[0] == 0xB5 && data[1] == 0x62)
		{
			// header found! wait for 4 more byte.
			int64_t t = systimer->gettime();
			while(uart->available() < 4)
				if (systimer->gettime() > timeout_tick)
					return NULL;				

			ubx_packet * pkt = &_packet;
				
			uart->read(&pkt->cls, 1);
			uart->read(&pkt->id, 1);
			uart->read(&pkt->payload_size, 2);
			
			// payload size found, wait for contents and crc
			while(uart->available() < pkt->payload_size+2)
				if (systimer->gettime() > timeout_tick)
					return NULL;

			pkt->payload = payload_buffer;
			uart->read(pkt->payload, pkt->payload_size);
			uart->read(&pkt->crc, 2);
				
			// check CRC
			uint8_t CK_A = 0;
			uint8_t CK_B = 0;
			for(int i=0; i<4; i++)
			{
				CK_A = CK_A + ((uint8_t*)pkt)[i];
				CK_B = CK_B + CK_A;
			}
			for(int i=0; i<pkt->payload_size; i++)
			{
				CK_A = CK_A + pkt->payload[i];
				CK_B = CK_B + CK_A;
			}

			pkt->crc_ok = pkt->crc == (CK_A | (CK_B<<8));
			
			t = systimer->gettime() - t;

			return pkt;
		}
	}while (systimer->gettime() < timeout_tick);

	return 0;
}

int UartUbloxNMEAGPS::init(HAL::IUART *uart, int baudrate)
{
	UartNMEAGPS::init(uart, baudrate);

	//

	int64_t t = systimer->gettime();
	int real_baudrate;
	if ((real_baudrate = set_baudrate(baudrate)) < 0)
	{
		printf("ublox GPS not found.\n");
		return -1;
	}
	t = systimer->gettime() - t;
	
	printf("ublox GPS initialized in %d us, baudrate set to %d.\n", int(t), real_baudrate);

	if (enable_message(0x1, 0x2) < 0)
		return -2;

	return 0;
}

}