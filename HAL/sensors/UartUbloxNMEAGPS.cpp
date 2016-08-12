#include "UartUbloxNMEAGPS.h"
#include <stdlib.h>
#include <stdio.h>
#include <HAL/Interface/ISystimer.h>
#include <protocol/common.h>
#include <utils/log.h>
#include <time.h>

#pragma pack(1)

namespace sensors
{

UartUbloxGPS::UartUbloxGPS()
{
	current_baudrate = 0;
	sat_config = IOCTL_SAT_NORMAL;
}

UartUbloxGPS::~UartUbloxGPS()
{

}

int UartUbloxGPS::wait_ack(int cls, int id, int timeout)
{
	int64_t timeout_tick = timeout>=0 ? (systimer->gettime() + timeout) : 0x7fffffffffffffff;
	ubx_packet *pkt;
	do
	{
		pkt = read_ubx_packet(timeout);

		if (pkt && pkt->cls == 0x5 && pkt->crc_ok && pkt->payload_size >= 2)
		{
			if (pkt->payload[0] == cls && pkt->payload[1] == id)
				return pkt->id == 1 ? 0 : 1;
		}
	}while(systimer->gettime() < timeout_tick);

	return -1;
}

int UartUbloxGPS::enable_message(uint8_t cls, uint8_t id, bool enable/* = true*/, int timeout /* = 100000 */)
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

int UartUbloxGPS::trig_ubx_packet(int timeout)
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

int UartUbloxGPS::set_baudrate(int baudrate)
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

int UartUbloxGPS::open_as(int baudrate)
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

int UartUbloxGPS::detect_baudrate()
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
		
		if (trig_ubx_packet(20000000000/baudtable[i]) == 0)
			return baudtable[i];
	}	
	
	return -1;	
}

int UartUbloxGPS::send_ubx_packet(uint8_t cls, uint8_t id, void *payload, uint16_t payload_size)
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

ubx_packet* UartUbloxGPS::read_ubx_packet(int timeout)
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

ubx_packet* UartUbloxGPS::read_ubx_packet()
{
	if (uart->available() <= 2)
		return NULL;

	uint8_t data[6];

	// search header "\xB5\x62"
	while(uart->available() > 6)
	{
		uart->peak(data, 2);

		if (data[0] != 0xB5 || data[1] != 0x62)
		{
			// not header, remove invalid data
			uart->read(data, data[1] == 0xB5 ? 1 : 2);
		}
		else
		{
			// 2byte B562, 4byte payload header, 2 byte CRC
			uart->peak(data, 6);
			int size = (int)data[5] << 8 | data[4];
			
			// fix: large buffer blockage
			if (size > 500)
			{
				uart->read(data, 6);
				continue;
			}

			if (uart->available() < size + 8)
				return NULL;

			ubx_packet * pkt = &_packet;
			uart->read(data, 2);	// B562
			uart->read(&pkt->cls, 1);
			uart->read(&pkt->id, 1);
			uart->read(&pkt->payload_size, 2);
			uart->read(pkt->payload, size);
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

			return pkt;
		}
	}

	return NULL;
}

int UartUbloxGPS::detect_and_config(HAL::IUART *uart, int baudrate)
{
	int64_t t = systimer->gettime();
	systimer->delayms(400);
	int real_baudrate;
	if ((real_baudrate = set_baudrate(baudrate)) < 0)
	{
		printf("ublox GPS not found.\n");
		return -1;
	}


	// NMEA config
	struct
	{
		uint8_t filter;
		uint8_t nmeaVersion;
		uint8_t numSV;
		uint8_t flags;
		uint32_t gnssToFilter;
		uint8_t svNumbering;
		uint8_t mainTalkerId;
		uint8_t gsvTalkerId;
		uint8_t version;
		char bdsTalkerId[2];
		uint8_t reserved[6];
	} nmea_cfg = 
	{
		0,		// no filter
		0x41,	// NMEA 4.1
		0,		// no limit
		2,		// consider mode
		0,		// no filter
		1,		// extended SV number
		1,		// main talker "GP"
		1,		// use main talker ID
		1,		// .
		"",		// default BeiDou talker
		{0},
	};

	int size = sizeof(nmea_cfg);
	send_ubx_packet(0x6, 0x17, &nmea_cfg, 20);
	if (wait_ack(0x6, 0x17) != 0)
		return -3;

	// NAV5 config
	ubx_nav5_config_packet nav_cfg = 
	{
		0xffff,
		6,
		3,
		0,
		10000,
		5,
		0,
		250,
		250,
		100,
		300,
		0,
		60,
		6,
		28,
		{0,0},
		200,
		0,
		{0,0,0,0,0}
	};
	_nav_cfg = nav_cfg;
	size = sizeof(nav_cfg);
	send_ubx_packet(0x6, 0x24, &nav_cfg, size);
	if (wait_ack(0x6, 0x24) != 0)
		return -3;
	last_nav_sending_time = systimer->gettime();
	nav_config_required = false;

	// rate config
	uint16_t rate_config[3] = {100, 1, 0};
	send_ubx_packet(0x6, 0x8, rate_config, 6);
	if (wait_ack(0x6, 0x8) != 0)
		return -4;

	// save settings for faster boot
	uint32_t save[3] = {0,0x1f,0};
	send_ubx_packet(0x6, 0x9, save, 12);
	if (wait_ack(0x6, 0x9) != 0)
		return -4;

	t = systimer->gettime() - t;

	printf("ublox GPS initialized in %d us, baudrate set to %d.\n", int(t), real_baudrate);

	return 0;
}


int UartUbloxNMEAGPS::init(HAL::IUART *uart, int baudrate)
{
	UartNMEAGPS::init(uart, baudrate);
	UartUbloxGPS::init(uart, baudrate);

	if (detect_and_config(uart, baudrate) < 0)
		return -1;

	// config messages

	// GPGGA
	if (enable_message(0xf0, 0x00, true) < 0)
		return -2;
	// GPGLL
	if (enable_message(0xf0, 0x01, true) < 0)
		return -2;
	// GPGSA
	if (enable_message(0xf0, 0x02, true) < 0)
		return -2;
	// GPGSV
	if (enable_message(0xf0, 0x03, true) < 0)
		return -2;
	// GPRMC
	if (enable_message(0xf0, 0x04, true) < 0)
		return -2;
	// GPVTG
	if (enable_message(0xf0, 0x05, true) < 0)
		return -2;
	// GPZDA
	if (enable_message(0xf0, 0x08, true) < 0)
		return -2;

	// UBX-POSLLH
	if (enable_message(0x01, 0x02, false) < 0)
		return -2;

	return 0;
}

UartUbloxBinaryGPS::UartUbloxBinaryGPS()
:UartUbloxGPS()
{
	next_nav_config_ioctl_time = 0;
}

UartUbloxBinaryGPS::~UartUbloxBinaryGPS()
{

}

int UartUbloxBinaryGPS::init(HAL::IUART *uart, int baudrate)
{
	UartUbloxGPS::init(uart, baudrate);

	if (detect_and_config(uart, baudrate) < 0)
		return -1;

	packt_mask = 0;

	// config messages

	// GPGGA
	if (enable_message(0xf0, 0x00, false) < 0)
		return -2;
	// GPGLL
	if (enable_message(0xf0, 0x01, false) < 0)
		return -2;
	// GPGSA
	if (enable_message(0xf0, 0x02, false) < 0)
		return -2;
	// GPGSV
	if (enable_message(0xf0, 0x03, false) < 0)
		return -2;
	// GPRMC
	if (enable_message(0xf0, 0x04, false) < 0)
		return -2;
	// GPVTG
	if (enable_message(0xf0, 0x05, false) < 0)
		return -2;
	// GPZDA
	if (enable_message(0xf0, 0x08, false) < 0)
		return -2;

	// UBX-PVT
	if (enable_message(0x01, 0x07) < 0)
		return -2;

	// UBX-DOP
	if (enable_message(0x01, 0x04) < 0)
		return -2;

	// NAV-POSLLH
	if (enable_message(0x01, 0x02, false) < 0)
		return -2;

	// UBX-SAT
	if (enable_message(0x01, 0x35, true) < 0)
		return -2;

	// save settings for faster boot
	uint32_t save[3] = {0,0x1f,0};
	send_ubx_packet(0x6, 0x9, save, 12);
	if (wait_ack(0x6, 0x9) != 0)
		return -4;


	return 0;
}
int UartUbloxBinaryGPS::ioctl(int request, void *data)
{
	if (request == sat_config || !(request == IOCTL_SAT_NORMAL || request == IOCTL_SAT_MINIMUM))
		return 0;

	if (systimer->gettime() < next_nav_config_ioctl_time)
		return 1;

	sat_config = request;
	next_nav_config_ioctl_time = systimer->gettime() + 1000000;

	if (sat_config == IOCTL_SAT_NORMAL)
	{
		_nav_cfg.cno_thresh = 28;
		_nav_cfg.cno_thresh_num_SVs = 6;
	}
	else
	{
		_nav_cfg.cno_thresh = 0;
		_nav_cfg.cno_thresh_num_SVs = 0;
	}

	nav_config_required = true;
	last_nav_sending_time = 0;

	if (request == IOCTL_SAT_NORMAL)
		LOGE("ublox GPS: switching to normal operation, nav=%d*%d\n", _nav_cfg.cno_thresh_num_SVs, _nav_cfg.cno_thresh);
	if (request == IOCTL_SAT_MINIMUM)
		LOGE("ublox GPS: switching to minimum operation\n");
	

	return 0;
}

int UartUbloxBinaryGPS::read(devices::gps_data *data)
{
	ubx_packet *p = NULL;

	int t2 = systimer->gettime();

	// send nav config packet @ 5hz if new nav5 configuration required.
	if (nav_config_required && systimer->gettime() > last_nav_sending_time + 200000)
	{
		send_ubx_packet(0x6, 0x24, &_nav_cfg, sizeof(_nav_cfg));
		last_nav_sending_time = systimer->gettime();
	}

	while(true)
	{
		int64_t t = systimer->gettime();
		p = read_ubx_packet();
		t = systimer->gettime() - t;

		if (!p)
			break;

		// check for NAV5 ACK packet
		if (p->cls == 0x5 && p->crc_ok && p->payload_size >= 2)
		{
			if (p->payload[0] == 0x6 && p->payload[1] == 0x24)
			{
				nav_config_required = false;
				LOGE("UBX:NAV5 ACK\n");
			}
		}

// 		printf("packet:%2x, %2x, CRC(%s)\n", p->cls, p->id, p->crc_ok ? "OK" : "FAIL");

		if (p->crc_ok)
		{
			// UBX-PVT(Position, Velocity, Time)
			// time, fix type & flag
			// pos LLH, velocity ned
			// speed and heading over ground
			// PDOP, sv used
			if (p->cls == 0x1 && p->id == 0x7)
			{
				typedef struct PVT_struct
				{
					uint32_t tow;
					uint16_t year;
					uint8_t month;
					uint8_t day;
					uint8_t hour;
					uint8_t min;
					uint8_t sec;
					uint8_t valid;
					uint32_t time_accuracy;
					int32_t nano;
					uint8_t fix;
					uint8_t flags;
					uint8_t reserved1;
					uint8_t sv_used;
					int32_t lon;
					int32_t lat;
					int32_t height;
					int32_t hMSL;
					uint32_t horizontal_accuracy;
					uint32_t vertical_accuracy;
					int32_t velocity[4];	// [north, east, down, ground]
					int32_t heading;
					uint32_t velocity_accuracy;
					uint32_t heading_accuracy;
					uint16_t PDOP;
					uint8_t reserved2[6];
					int32_t heading_vehicle;
					uint8_t reserved3[4];
				} PVT;

				if (p->payload_size < sizeof(PVT))
					continue;

				PVT * pvt = (PVT *) p->payload;

				local_data.longitude = pvt->lon * (1E-7);		// longitude in degree
				local_data.latitude = pvt->lat * (1E-7);		// latitude in degree
				local_data.speed = sqrt((float)pvt->velocity[0] * pvt->velocity[0] + pvt->velocity[1] * pvt->velocity[1]) / 1000.0f;			// unit: meter/s
				local_data.altitude = pvt->height/1000.0f;					// meter
				local_data.direction = atan2((float)pvt->velocity[1], (float)pvt->velocity[0]) * 180 / PI;			// Track angle in degrees True, 0-360 degree, 0: north, 90: east, 180: south, 270: west, 359:almost north
				local_data.climb_rate = pvt->velocity[2] / -1000.0f;
				if (local_data.direction < 0)
					local_data.direction += 360;
				local_data.DOP[0] = pvt->PDOP;				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
// 				local_data.DOP[1] = info.HDOP*100;				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
// 				local_data.DOP[2] = info.VDOP*100;				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
				local_data.satelite_in_view = pvt->sv_used;
				local_data.satelite_in_use = pvt->sv_used;
				local_data.sig = pvt->fix>=3 ? 1 : 0;						// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
				local_data.fix = pvt->fix;						// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
				local_data.declination = 0;// Magnetic variation in 0.01 degrees (Easterly var. subtracts from true course)
				local_data.position_accuracy_horizontal = pvt->horizontal_accuracy / 1000.0f;
				local_data.position_accuracy_vertical = pvt->vertical_accuracy / 1000.0f;
				local_data.velocity_accuracy_horizontal = pvt->velocity_accuracy / 1000.0f;
				local_data.velocity_accuracy_vertical = NAN;	// NAN: not available / undefined;

				log2(p->payload, TAG_UBX_NAV_PVT_DATA, p->payload_size);

				struct tm _tm;	
				_tm.tm_sec = pvt->sec;
				_tm.tm_min = pvt->min;
				_tm.tm_hour = pvt->hour;
				_tm.tm_mday = pvt->day;
				_tm.tm_mon = pvt->month - 1;
				_tm.tm_year = pvt->year - 1900;	

				local_data.timestamp = mktime(&_tm);


				packt_mask |= nav_pvt;
			}

			// UBX-DOP
			else if (p->cls == 0x1 && p->id == 0x4)
			{
				typedef struct DOP_struct
				{
					uint32_t tow;
					uint16_t GDOP;
					uint16_t PDOP;
					uint16_t TDOP;
					uint16_t VDOP;
					uint16_t HDOP;
					uint16_t NDOP;
					uint16_t EDOP;
				} DOP;

				if (p->payload_size < sizeof(DOP))
					continue;

				DOP * dop = (DOP *)p->payload;

				local_data.DOP[0] = dop->PDOP;
				local_data.DOP[1] = dop->HDOP;
				local_data.DOP[2] = dop->VDOP;

				packt_mask |= nav_dop;
			}

			// UBX-SAT
			// sat visible
			else if (p->cls == 0x1 && p->id == 0x35)
			{
				typedef struct SAT_struct
				{
					uint8_t gnss_id;
					uint8_t sv_id;
					uint8_t cno;
					int8_t elev;
					int16_t azim;
					int16_t residual;
					uint32_t flags;
				} SAT;

				typedef struct SAT_header_struct
				{
					uint32_t tow;
					uint8_t version;
					uint8_t num_sat_visible;
					uint8_t resv[2];
					SAT sats[20];
				} SAT_header;

				SAT_header *sat = (SAT_header*) p->payload;

				/*
				printf("num SAT:%d\n", sat->num_sat_visible);
				int visible = 0;
				for(int i=0; i<sat->num_sat_visible; i++)
				{
					printf("sv:%d, CNO:%d, flag:%x\n", sat->sats[i].sv_id, sat->sats[i].cno, sat->sats[i].flags & 7);

					if ((sat->sats[i].flags & 7) >= 3)
						visible++;
				}

				printf("total visible: %d\n", visible);
				*/
				log2(p->payload, TAG_UBX_SAT_DATA, p->payload_size);



				packt_mask |= nav_sat;
			}

			else
			{
				TRACE("unexpected UBX packet %02X,%02X\n", p->cls, p->id);
			}
		}
	}

// 	t2 = systimer->gettime() - t2;
// 	printf("t2 cost:%d\n", int(t2));

	// have we collected pvt and dop packets?
	if (packt_mask & (nav_pvt | nav_dop) == (nav_pvt | nav_dop))
	{
		*data = local_data;

		TRACE("UBX data: t=%f, mask=%02x\n", systimer->gettime()/1000000.0f, packt_mask);
		log2(&local_data, TAG_UBX_0_DATA, sizeof(local_data));
		packt_mask = 0;

		return 0;
	}

	return 1;
}

}

