#include "ZLinkWifi.h"
#include <modules/utils/crc16.h>
#include <string.h>

void update_crc(ZLinkWifiPacket * pkt)
{
	pkt->magic = ZLINKWIFI_MAGIC;
	pkt->crc = crc16((unsigned char*)&(pkt->size), sizeof(ZLinkWifiPacket) - 4 + pkt->size);
	uint16_t crc = crc16((unsigned char*)&(pkt->size), sizeof(ZLinkWifiPacket) - 4 + pkt->size);

}

bool check_crc(ZLinkWifiPacket *pkt)
{
	uint16_t crc = crc16((unsigned char*)&(pkt->size), sizeof(ZLinkWifiPacket) - 4 + pkt->size);

	if (crc != pkt->crc)
		return false;

	return (pkt->magic == ZLINKWIFI_MAGIC) && (pkt->crc == crc);
}

int search_pkt(HAL::IUART *uart, ZLinkWifiPacket *pkt)
{
	uint8_t header0 = ZLINKWIFI_MAGIC & 0xff;
	uint8_t header1 = (ZLINKWIFI_MAGIC >> 8) & 0xff;

	while (uart->available() >= sizeof(ZLinkWifiPacket))
	{
		// check head
		uint8_t header[2];
		uart->peak(header, 2);
		if (header[0] != header0 || header[1] != header1)
		{
			uart->read(header, 1);
			continue;
		}

		// check body size
		uart->peak(pkt, sizeof(ZLinkWifiPacket));
		if (pkt->size > MAX_ZLINK_WIFI_PKT_SIZE)
		{
			uart->read(header, 2);
			continue;
		}

		if (uart->available() < sizeof(ZLinkWifiPacket) + pkt->size)
			return -1;
		
		uart->peak(pkt, sizeof(ZLinkWifiPacket) + pkt->size);
		if (check_crc(pkt))
		{
			uart->read(pkt, sizeof(ZLinkWifiPacket) + pkt->size);
			return sizeof(ZLinkWifiPacket) + pkt->size;
		}
		else
		{
			uart->read(header, 2);
		}
	}

	return 0;
}

void fill_pkt(ZLinkWifiPacket *pkt, uint8_t type, uint8_t *data, int size)
{
	pkt->type = type;
	pkt->size = size;
	memcpy(pkt->data, data, size);
	update_crc(pkt);
}
