#pragma once

#include "UartNMEAGPS.h"

namespace sensors
{
	static const int IOCTL_SAT_NORMAL = 0x38;	// set satelite requirements to normal operation, to reject low quality sats to avoid drifting.
	static const int IOCTL_SAT_MINIMUM = 0x39;	// set satelite requirements to minimum, to achieve max RTL possibility in loss of controll situation.
	typedef struct
	{
		uint8_t cls;
		uint8_t id;
		uint16_t payload_size;
		uint8_t *payload;
		uint16_t crc;
		bool crc_ok;
	} ubx_packet;
	typedef struct
	{
		uint16_t mask;
		uint8_t dyn_model;
		uint8_t fix_mode;
		int32_t fix_alt;
		uint32_t fix_alt_var;
		uint8_t min_elev;
		uint8_t dr_limit;
		uint16_t pdop;
		uint16_t tdop;
		uint16_t pacc;
		uint16_t tacc;
		uint8_t static_hold_thresh;
		uint8_t dgps_timeout;
		uint8_t cno_thresh_num_SVs;
		uint8_t cno_thresh;
		uint8_t reserved1[2];
		uint16_t static_hold_max_dist;
		uint8_t utc_standard;
		uint8_t reserved2[5];
	} ubx_nav5_config_packet;

	class UartUbloxGPS
	{
	public:
		UartUbloxGPS();
		~UartUbloxGPS();

		void init(HAL::IUART *uart, int baudrate){this->uart = uart; current_baudrate = baudrate;}

	protected:

		// return pointer to local ubx_packet if valid ubx_packet found in UART buffer, NULL if not found.
		//   will return immediatly if ubx header not found in UART buffer, regardless of timeout setting
		//   will wait for content if ubx header found, until timeout.
		//   pass -1 for it to wait infinitly
		ubx_packet* read_ubx_packet(int timeout);

		// return pointer to local ubx_packet if valid ubx_packet found in UART buffer, NULL if not found.
		//   will return immediatly
		ubx_packet* read_ubx_packet();

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

		// detect and config GNSS settings
		// return negative value on error.
		int detect_and_config(HAL::IUART *uart, int baudrate);

		uint8_t payload_buffer[512];
		ubx_packet _packet;
		int current_baudrate;
		HAL::IUART *uart;

		// nav5 setting sender
		int sat_config;
		ubx_nav5_config_packet _nav_cfg;
		int64_t last_nav_sending_time;
		bool nav_config_required;
	};

	class UartUbloxNMEAGPS : public UartUbloxGPS, public UartNMEAGPS
	{
	public:
		int init(HAL::IUART *uart, int baudrate);
	};

	class UartUbloxBinaryGPS : public UartUbloxGPS, public devices::IGPS, public devices::IRawDevice
	{
	public:
		UartUbloxBinaryGPS();
		~UartUbloxBinaryGPS();

		int init(HAL::IUART *uart, int baudrate);
		virtual int read(devices::gps_data *data);
		virtual bool healthy(){return true;}
		virtual int ioctl(int request, void *data);

	protected:
		enum ubx_nav_packets
		{
			nav_pvt = 1,
			nav_dop = 2,
			nav_sat = 4,
		};

		uint32_t packt_mask;
		devices::gps_data local_data;
		int64_t next_nav_config_ioctl_time;
	};
}
