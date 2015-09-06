#pragma once

#include "UartNMEAGPS.h"

namespace sensors
{
	typedef struct
	{
		uint8_t cls;
		uint8_t id;
		uint16_t payload_size;
		uint8_t *payload;
		uint16_t crc;
		bool crc_ok;
	} ubx_packet;

	class UartUbloxNMEAGPS : public UartNMEAGPS
	{
	public:
		UartUbloxNMEAGPS();
		~UartUbloxNMEAGPS();

		int init(HAL::IUART *uart, int baudrate);

	protected:

		// return pointer to local ubx_packet if valid ubx_packet found in UART buffer, NULL if not found.
		//   will return immediatly if ubx header not found in UART buffer, regardless of timeout setting
		//   will wait for content if ubx header found, until timeout.
		//   pass -1 for it to wait infinitly
		ubx_packet* read_ubx_packet(int timeout);


		// try to enable a message type, or pass enable=false to disable it.
		//   return 0 if succeeded
		//   negative value for error.
		int enable_message(uint8_t cls, uint8_t id, bool enable = true, int timeout = 200000);


		// send a ubx packet, with header and crc.
		int send_ubx_packet(uint8_t cls, uint8_t id, void *payload, uint16_t payload_size);

		// try receive any ubx packet while triggerring a ubx "ACK" packet
		//   return -1 if timeout
		//   return 0 if success
		int trig_ubx_packet(int timeout);


		// try receive any ubx packet while triggerring a ubx "ACK" packet
		//   return -1 if timeout
		//   return 0 if ACK
		//   return 1 if NAK
		int wait_ack(int cls, int id, int timeout = 200000);


		// detect ublox baudrate.
		//   return baudrate if ubx response received.
		//   negative value if failed all possible baudrate.
		int detect_baudrate();

		// try set ublox baudrate.
		//    return new baudrate if succeeded
		//    negative value for error.
		int set_baudrate(int baudrate);

		// helper function that switch baudrate and flush all uart buffers
		int open_as(int baudrate);

		uint8_t payload_buffer[512];
		ubx_packet _packet;
		int current_baudrate;
	};

	enum receiver_state
	{
		wait_for_ubx_header,
		wait_for_packet_header,
		wait_for_content,
	};

	class UartUbloxBinaryGPS : public UartUbloxNMEAGPS
	{
	public:
		UartUbloxBinaryGPS();
		~UartUbloxBinaryGPS();

		int init(HAL::IUART *uart, int baudrate);

	protected:
		receiver_state state;
	};
}
