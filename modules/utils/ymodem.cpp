#include "ymodem.h"
#include <stdlib.h>
#include <HAL/Interface/ISysTimer.h>

#define SOH                     (0x01)  /* start of 128-byte data packet */
#define STX                     (0x02)  /* start of 1024-byte data packet */
#define ETX                     (0x03)  /* end of Text */
#define EOT                     (0x04)  /* end of transmission */
#define ACK                     (0x06)  /* acknowledge */
#define NAK                     (0x15)  /* negative acknowledge */
#define CA                      (0x18)  /* two of these in succession aborts transfer */
#define CRC16                   (0x43)  /* 'C' == 0x43, request 16-bit CRC */

#define ABORT1                  (0x41)  /* 'A' == 0x41, abort by user */
#define ABORT2                  (0x61)  /* 'a' == 0x61, abort by user */


static uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte)
{
    uint32_t crc = crcIn;
    uint32_t in = byte|0x100;
    do
    {
        crc <<= 1;
        in <<= 1;
        if (in&0x100)
            ++crc;
        if (crc&0x10000)
            crc ^= 0x1021;
    }
    while (!(in&0x10000));
    return crc&0xffffu;
}

static uint16_t crc16(const uint8_t* data, uint32_t size)
{
    uint32_t crc = 0;
    const uint8_t* dataEnd = data+size;
    while (data<dataEnd)
        crc = UpdateCRC16(crc,*data++);

    crc = UpdateCRC16(crc,0);
    crc = UpdateCRC16(crc,0);
    return crc&0xffffu;
}

ymodem_receiver::ymodem_receiver(HAL::IUART *uart)
{
	this->uart=uart;
	timeout = 10000000;
	reset();
}

ymodem_receiver::~ymodem_receiver()
{

}

int ymodem_receiver::reset()
{
	// clear uart buffer
	char tmp[1024];
	int got = 0;
	while(got >=0)
		got = uart->read(tmp, 1024);

	state = ymodem_wait_file_header;
	Ctimeout = systimer->gettime();

	return 0;
}

int ymodem_receiver::run()
{
	switch(state)
	{
	case ymodem_wait_file_header:
		{
			// 'C' character sending
			if (Ctimeout > 0 && systimer->gettime() > Ctimeout)
			{
				Ctimeout = systimer->gettime() + 1000000;
				uart->write("C", 1);
			}

			// wait for SOH byte
			uint8_t tmp = 0;
			if (uart->read(&tmp, 1) == 1 && tmp == SOH)
			{
				// stop "C"
				Ctimeout = 0;

				// treat next 2 + 128 + 2 (0x00 0xff [128 byte data] crch crcl) bytes as file header packet
				// wait for enough bytes or timeout.
				int64_t wait_end = systimer->gettime() + timeout;
				while(uart->available() < 132)
				{
					if (systimer->gettime() > wait_end)
					{
						on_event(NULL, 0, ymodem_error);
						state = ymodem_state_error;
						break;
					}
				}

				// check header and CRC
				uart->read(data, 132);
				uint16_t crc_received = (data[130] << 8) | data[131];
				uint16_t crc_calculated = crc16(data+2, 128);
				if (data[0] != (~(data[1]) & 0xff) || crc_received != crc_calculated)
				{
					uint8_t b = NAK;
					uart->write(&b, 1);

					// drop this packet
					on_event(data+2, 128, ymodem_bad_packet);
					if (error_count++ > 10)
					{
						on_event(NULL, 0, ymodem_error);
						state = ymodem_state_error;
						break;
					}

					break;
				}

				// state switching and event delivering
				uint8_t b = ACK;
				uart->write(&b, 1);
				on_event(data+2, 128, ymodem_file_header);
				state = ymodem_wait_packets;
				Ctimeout = systimer->gettime();
				blk_number = 1;
			}
		}
		break;
	case ymodem_wait_packets:
		{
			// 'C' character sending
			if (Ctimeout > 0 && systimer->gettime() > Ctimeout)
			{
				Ctimeout = systimer->gettime() + 1000000;
				uart->write("C", 1);
			}

			// wait for SOH/STX/EOT byte
			uint8_t tmp = 0;
			if (uart->read(&tmp, 1) == 1 && (tmp == SOH || tmp == STX || tmp == EOT))
			{
				// stop "C" character
				Ctimeout = 0;

				// EOT checking
				if (tmp == EOT)
				{
					// NAK once and check next byte
					uint8_t b = NAK;
					uart->write(&b, 1);

					int64_t wait_end = systimer->gettime() + timeout;
					while(uart->available() < 1)
						if (systimer->gettime() > wait_end)
							break;

					uart->read(&b, 1);
					if (b == EOT)
					{
						b = ACK;
						uart->write(&b, 1);
						state = ymodem_transition_ended;
						on_event(NULL, 0, ymodem_end_of_transition);	
						break;
					}
				}

				// treat next 2 + data_size + 2 (0x00 0xff [N byte data] crch crcl) bytes as file header packet
				// wait for enough bytes or timeout.
				int packet_size = (tmp == SOH ? 128 : 1024) + 4;
				int64_t wait_end = systimer->gettime() + timeout;
				while(uart->available() < packet_size)
				{
					if (systimer->gettime() > wait_end)
					{
						on_event(NULL, 0, ymodem_error);
						state = ymodem_state_error;
						break;
					}
				}

				// check header and CRC
				uart->read(data, packet_size);
				uint16_t crc_received = (data[packet_size-2] << 8) | data[packet_size-1];
				uint16_t crc_calculated = crc16(data+2, packet_size-4);
				if (blk_number != data[0] || data[0] != (~(data[1]) & 0xff) || crc_received != crc_calculated)
				{
					uint8_t b = NAK;
					uart->write(&b, 1);

					// drop this packet
					on_event(data+2, packet_size-4, ymodem_bad_packet);
					if (error_count++ > 10)
					{
						on_event(NULL, 0, ymodem_error);
						state = ymodem_state_error;
						break;
					}

					break;
				}

				// packet accepted
				uint8_t b = ACK;
				uart->write(&b, 1);
				blk_number++;
				on_event(data+2, packet_size-4, ymodem_file_data);
			}
		}
		break;
	case ymodem_transition_ended:
		break;
	case ymodem_state_error:
		break;
	default:
		reset();
		break;
	}
	
	return state;
}
