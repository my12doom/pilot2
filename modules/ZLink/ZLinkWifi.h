#pragma once

#include <stdint.h>
#include <HAL/Interface/IUART.h>

#define ZLINKWIFI_MAGIC 0x85A4
#define MAX_ZLINK_WIFI_PKT_SIZE 512


#pragma pack(push, 1)

typedef struct 
{
    uint16_t magic;
    uint16_t crc;
    uint16_t size;
    uint8_t type;
    uint8_t data[0];
} ZLinkWifiPacket;

#pragma pack(pop)


enum ZLinkWifiType
{
	ZLinkWifiType_TCP = 1,
	ZLinkWifiType_UDP = 2,
};

int search_pkt(HAL::IUART *uart, ZLinkWifiPacket *pkt);
void update_crc(ZLinkWifiPacket * pkt);
bool check_crc(ZLinkWifiPacket *pkt);
void fill_pkt(ZLinkWifiPacket *pkt, uint8_t type, uint8_t *data, int size);