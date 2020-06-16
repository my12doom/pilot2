#pragma once
#pragma pack(push, 1)

#include <stdint.h>

namespace hover {
typedef struct
{
    uint8_t magic;              ///< protocol magic marker
    uint8_t len;                ///< Length of payload
    uint8_t seq;                ///< Sequence of packet
    uint8_t sysid;              ///< ID of message sender system/aircraft
    uint8_t compid;             ///< ID of the message sender component
    uint8_t msgid;          ///< first 8 bits of the ID of the message
    uint8_t payload[0];   ///< A maximum of 255 payload bytes
    //uint16_t checksum;          ///< X.25 CRC
} mavlink_header;

typedef struct
{
    int32_t x; /*< [m] X Position in NED frame*/
    int32_t y; /*< [m] Y Position in NED frame*/
    float z; /*< [m] Z Position in NED frame (note, altitude is negative in NED)*/
} ned_block_t;

typedef struct __mavlink_set_position_target_local_ned_t {
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    float x; /*< [m] X Position in NED frame*/
    float y; /*< [m] Y Position in NED frame*/
    float z; /*< [m] Z Position in NED frame (note, altitude is negative in NED)*/
    float vx; /*< [m/s] X velocity in NED frame*/
    float vy; /*< [m/s] Y velocity in NED frame*/
    float vz; /*< [m/s] Z velocity in NED frame*/
    float afx; /*< [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
    float afy; /*< [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
    float afz; /*< [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
    float yaw; /*< [rad] yaw setpoint*/
    float yaw_rate; /*< [rad/s] yaw rate setpoint*/
    uint16_t type_mask; /*<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/
    uint8_t target_system; /*<  System ID*/
    uint8_t target_component; /*<  Component ID*/
    uint8_t coordinate_frame; /*<  Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9*/
} mavlink_set_position_target_local_ned_t;

typedef struct __mavlink_set_position_target_wgs84_local_ned_t {
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    int32_t x; /*< [m] X Position in NED frame*/
    int32_t y; /*< [m] Y Position in NED frame*/
    float z; /*< [m] Z Position in NED frame (note, altitude is negative in NED)*/
    float vx; /*< [m/s] X velocity in NED frame*/
    float vy; /*< [m/s] Y velocity in NED frame*/
    float vz; /*< [m/s] Z velocity in NED frame*/
    float afx; /*< [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
    float afy; /*< [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
    float afz; /*< [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
    float yaw; /*< [rad] yaw setpoint*/
    float yaw_rate; /*< [rad/s] yaw rate setpoint*/
    uint16_t type_mask; /*<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/
    uint8_t target_system; /*<  System ID*/
    uint8_t target_component; /*<  Component ID*/
    uint8_t coordinate_frame; /*<  Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9*/
} mavlink_set_position_target_wgs84_local_ned_t;

#pragma pack(pop)


static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	/*Accumulate one byte of data into the CRC*/
	uint8_t tmp;

	tmp = data ^ (uint8_t)(*crcAccum & 0xff);
	tmp ^= (tmp << 4);
	*crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static inline uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
	uint16_t crcTmp = 0xffff;
	while (length--) {
		crc_accumulate(*pBuffer++, &crcTmp);
	}
	return crcTmp;
}
static inline void crc_init(uint16_t* crcAccum)
{
        *crcAccum = 0xFFFF;
}
}