#pragma once
// MESSAGE SWARM_DRONE_DATA PACKING

#define MAVLINK_MSG_ID_SWARM_DRONE_DATA 94

MAVPACKED(
typedef struct __mavlink_swarm_drone_data_t {
 uint64_t firmware_version; /*<  firmware_version(example:PX4_GIT_VERSION_BINARY 0x38aaaf2cef9c1098)*/
 int32_t latitude; /*<  latitude*/
 int32_t longitude; /*<  longitude*/
 int32_t gps_altitude; /*<  Altitude from the ground*/
 uint32_t onboard_control_sensors_health; /*<  Check drone status((Value of 0: error. Value of 1: healthy.)bit map https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR )*/
 float yaw; /*<  drone yaw(units deg 0-360)*/
 uint8_t RTK_fix_type; /*<  RTK fix type(0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed)*/
 int8_t battery_remaining; /*<  Remaining battery energy(0 : 0%, 100 : 100%, -1 : no battery)*/
}) mavlink_swarm_drone_data_t;

#define MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN 30
#define MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN 30
#define MAVLINK_MSG_ID_94_LEN 30
#define MAVLINK_MSG_ID_94_MIN_LEN 30

#define MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC 137
#define MAVLINK_MSG_ID_94_CRC 137



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SWARM_DRONE_DATA { \
    94, \
    "SWARM_DRONE_DATA", \
    8, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_swarm_drone_data_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_swarm_drone_data_t, longitude) }, \
         { "gps_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_swarm_drone_data_t, gps_altitude) }, \
         { "RTK_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_swarm_drone_data_t, RTK_fix_type) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 29, offsetof(mavlink_swarm_drone_data_t, battery_remaining) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_swarm_drone_data_t, firmware_version) }, \
         { "onboard_control_sensors_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_swarm_drone_data_t, onboard_control_sensors_health) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_swarm_drone_data_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SWARM_DRONE_DATA { \
    "SWARM_DRONE_DATA", \
    8, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_swarm_drone_data_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_swarm_drone_data_t, longitude) }, \
         { "gps_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_swarm_drone_data_t, gps_altitude) }, \
         { "RTK_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_swarm_drone_data_t, RTK_fix_type) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 29, offsetof(mavlink_swarm_drone_data_t, battery_remaining) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_swarm_drone_data_t, firmware_version) }, \
         { "onboard_control_sensors_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_swarm_drone_data_t, onboard_control_sensors_health) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_swarm_drone_data_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a swarm_drone_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  latitude
 * @param longitude  longitude
 * @param gps_altitude  Altitude from the ground
 * @param RTK_fix_type  RTK fix type(0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed)
 * @param battery_remaining  Remaining battery energy(0 : 0%, 100 : 100%, -1 : no battery)
 * @param firmware_version  firmware_version(example:PX4_GIT_VERSION_BINARY 0x38aaaf2cef9c1098)
 * @param onboard_control_sensors_health  Check drone status((Value of 0: error. Value of 1: healthy.)bit map https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR )
 * @param yaw  drone yaw(units deg 0-360)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_drone_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t gps_altitude, uint8_t RTK_fix_type, int8_t battery_remaining, uint64_t firmware_version, uint32_t onboard_control_sensors_health, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN];
    _mav_put_uint64_t(buf, 0, firmware_version);
    _mav_put_int32_t(buf, 8, latitude);
    _mav_put_int32_t(buf, 12, longitude);
    _mav_put_int32_t(buf, 16, gps_altitude);
    _mav_put_uint32_t(buf, 20, onboard_control_sensors_health);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, RTK_fix_type);
    _mav_put_int8_t(buf, 29, battery_remaining);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN);
#else
    mavlink_swarm_drone_data_t packet;
    packet.firmware_version = firmware_version;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.gps_altitude = gps_altitude;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.yaw = yaw;
    packet.RTK_fix_type = RTK_fix_type;
    packet.battery_remaining = battery_remaining;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_DRONE_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC);
}

/**
 * @brief Pack a swarm_drone_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  latitude
 * @param longitude  longitude
 * @param gps_altitude  Altitude from the ground
 * @param RTK_fix_type  RTK fix type(0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed)
 * @param battery_remaining  Remaining battery energy(0 : 0%, 100 : 100%, -1 : no battery)
 * @param firmware_version  firmware_version(example:PX4_GIT_VERSION_BINARY 0x38aaaf2cef9c1098)
 * @param onboard_control_sensors_health  Check drone status((Value of 0: error. Value of 1: healthy.)bit map https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR )
 * @param yaw  drone yaw(units deg 0-360)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_drone_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,int32_t gps_altitude,uint8_t RTK_fix_type,int8_t battery_remaining,uint64_t firmware_version,uint32_t onboard_control_sensors_health,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN];
    _mav_put_uint64_t(buf, 0, firmware_version);
    _mav_put_int32_t(buf, 8, latitude);
    _mav_put_int32_t(buf, 12, longitude);
    _mav_put_int32_t(buf, 16, gps_altitude);
    _mav_put_uint32_t(buf, 20, onboard_control_sensors_health);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, RTK_fix_type);
    _mav_put_int8_t(buf, 29, battery_remaining);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN);
#else
    mavlink_swarm_drone_data_t packet;
    packet.firmware_version = firmware_version;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.gps_altitude = gps_altitude;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.yaw = yaw;
    packet.RTK_fix_type = RTK_fix_type;
    packet.battery_remaining = battery_remaining;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_DRONE_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC);
}

/**
 * @brief Encode a swarm_drone_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param swarm_drone_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_drone_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_swarm_drone_data_t* swarm_drone_data)
{
    return mavlink_msg_swarm_drone_data_pack(system_id, component_id, msg, swarm_drone_data->latitude, swarm_drone_data->longitude, swarm_drone_data->gps_altitude, swarm_drone_data->RTK_fix_type, swarm_drone_data->battery_remaining, swarm_drone_data->firmware_version, swarm_drone_data->onboard_control_sensors_health, swarm_drone_data->yaw);
}

/**
 * @brief Encode a swarm_drone_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param swarm_drone_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_drone_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_swarm_drone_data_t* swarm_drone_data)
{
    return mavlink_msg_swarm_drone_data_pack_chan(system_id, component_id, chan, msg, swarm_drone_data->latitude, swarm_drone_data->longitude, swarm_drone_data->gps_altitude, swarm_drone_data->RTK_fix_type, swarm_drone_data->battery_remaining, swarm_drone_data->firmware_version, swarm_drone_data->onboard_control_sensors_health, swarm_drone_data->yaw);
}

/**
 * @brief Send a swarm_drone_data message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  latitude
 * @param longitude  longitude
 * @param gps_altitude  Altitude from the ground
 * @param RTK_fix_type  RTK fix type(0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed)
 * @param battery_remaining  Remaining battery energy(0 : 0%, 100 : 100%, -1 : no battery)
 * @param firmware_version  firmware_version(example:PX4_GIT_VERSION_BINARY 0x38aaaf2cef9c1098)
 * @param onboard_control_sensors_health  Check drone status((Value of 0: error. Value of 1: healthy.)bit map https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR )
 * @param yaw  drone yaw(units deg 0-360)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_swarm_drone_data_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, int32_t gps_altitude, uint8_t RTK_fix_type, int8_t battery_remaining, uint64_t firmware_version, uint32_t onboard_control_sensors_health, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN];
    _mav_put_uint64_t(buf, 0, firmware_version);
    _mav_put_int32_t(buf, 8, latitude);
    _mav_put_int32_t(buf, 12, longitude);
    _mav_put_int32_t(buf, 16, gps_altitude);
    _mav_put_uint32_t(buf, 20, onboard_control_sensors_health);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, RTK_fix_type);
    _mav_put_int8_t(buf, 29, battery_remaining);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_DRONE_DATA, buf, MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC);
#else
    mavlink_swarm_drone_data_t packet;
    packet.firmware_version = firmware_version;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.gps_altitude = gps_altitude;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.yaw = yaw;
    packet.RTK_fix_type = RTK_fix_type;
    packet.battery_remaining = battery_remaining;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_DRONE_DATA, (const char *)&packet, MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC);
#endif
}

/**
 * @brief Send a swarm_drone_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_swarm_drone_data_send_struct(mavlink_channel_t chan, const mavlink_swarm_drone_data_t* swarm_drone_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_swarm_drone_data_send(chan, swarm_drone_data->latitude, swarm_drone_data->longitude, swarm_drone_data->gps_altitude, swarm_drone_data->RTK_fix_type, swarm_drone_data->battery_remaining, swarm_drone_data->firmware_version, swarm_drone_data->onboard_control_sensors_health, swarm_drone_data->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_DRONE_DATA, (const char *)swarm_drone_data, MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_swarm_drone_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, int32_t gps_altitude, uint8_t RTK_fix_type, int8_t battery_remaining, uint64_t firmware_version, uint32_t onboard_control_sensors_health, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, firmware_version);
    _mav_put_int32_t(buf, 8, latitude);
    _mav_put_int32_t(buf, 12, longitude);
    _mav_put_int32_t(buf, 16, gps_altitude);
    _mav_put_uint32_t(buf, 20, onboard_control_sensors_health);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, RTK_fix_type);
    _mav_put_int8_t(buf, 29, battery_remaining);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_DRONE_DATA, buf, MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC);
#else
    mavlink_swarm_drone_data_t *packet = (mavlink_swarm_drone_data_t *)msgbuf;
    packet->firmware_version = firmware_version;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->gps_altitude = gps_altitude;
    packet->onboard_control_sensors_health = onboard_control_sensors_health;
    packet->yaw = yaw;
    packet->RTK_fix_type = RTK_fix_type;
    packet->battery_remaining = battery_remaining;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_DRONE_DATA, (const char *)packet, MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN, MAVLINK_MSG_ID_SWARM_DRONE_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE SWARM_DRONE_DATA UNPACKING


/**
 * @brief Get field latitude from swarm_drone_data message
 *
 * @return  latitude
 */
static inline int32_t mavlink_msg_swarm_drone_data_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field longitude from swarm_drone_data message
 *
 * @return  longitude
 */
static inline int32_t mavlink_msg_swarm_drone_data_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field gps_altitude from swarm_drone_data message
 *
 * @return  Altitude from the ground
 */
static inline int32_t mavlink_msg_swarm_drone_data_get_gps_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field RTK_fix_type from swarm_drone_data message
 *
 * @return  RTK fix type(0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed)
 */
static inline uint8_t mavlink_msg_swarm_drone_data_get_RTK_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field battery_remaining from swarm_drone_data message
 *
 * @return  Remaining battery energy(0 : 0%, 100 : 100%, -1 : no battery)
 */
static inline int8_t mavlink_msg_swarm_drone_data_get_battery_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  29);
}

/**
 * @brief Get field firmware_version from swarm_drone_data message
 *
 * @return  firmware_version(example:PX4_GIT_VERSION_BINARY 0x38aaaf2cef9c1098)
 */
static inline uint64_t mavlink_msg_swarm_drone_data_get_firmware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field onboard_control_sensors_health from swarm_drone_data message
 *
 * @return  Check drone status((Value of 0: error. Value of 1: healthy.)bit map https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR )
 */
static inline uint32_t mavlink_msg_swarm_drone_data_get_onboard_control_sensors_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field yaw from swarm_drone_data message
 *
 * @return  drone yaw(units deg 0-360)
 */
static inline float mavlink_msg_swarm_drone_data_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a swarm_drone_data message into a struct
 *
 * @param msg The message to decode
 * @param swarm_drone_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_swarm_drone_data_decode(const mavlink_message_t* msg, mavlink_swarm_drone_data_t* swarm_drone_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    swarm_drone_data->firmware_version = mavlink_msg_swarm_drone_data_get_firmware_version(msg);
    swarm_drone_data->latitude = mavlink_msg_swarm_drone_data_get_latitude(msg);
    swarm_drone_data->longitude = mavlink_msg_swarm_drone_data_get_longitude(msg);
    swarm_drone_data->gps_altitude = mavlink_msg_swarm_drone_data_get_gps_altitude(msg);
    swarm_drone_data->onboard_control_sensors_health = mavlink_msg_swarm_drone_data_get_onboard_control_sensors_health(msg);
    swarm_drone_data->yaw = mavlink_msg_swarm_drone_data_get_yaw(msg);
    swarm_drone_data->RTK_fix_type = mavlink_msg_swarm_drone_data_get_RTK_fix_type(msg);
    swarm_drone_data->battery_remaining = mavlink_msg_swarm_drone_data_get_battery_remaining(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN? msg->len : MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN;
        memset(swarm_drone_data, 0, MAVLINK_MSG_ID_SWARM_DRONE_DATA_LEN);
    memcpy(swarm_drone_data, _MAV_PAYLOAD(msg), len);
#endif
}
