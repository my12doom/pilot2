/** @file
 *    @brief MAVLink comm protocol testsuite generated from hover.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef HOVER_TESTSUITE_H
#define HOVER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_hover(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_hover(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_swarm_drone_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SWARM_DRONE_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_swarm_drone_data_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,185.0,89,156
    };
    mavlink_swarm_drone_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.firmware_version = packet_in.firmware_version;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.gps_altitude = packet_in.gps_altitude;
        packet1.onboard_control_sensors_health = packet_in.onboard_control_sensors_health;
        packet1.yaw = packet_in.yaw;
        packet1.RTK_fix_type = packet_in.RTK_fix_type;
        packet1.battery_remaining = packet_in.battery_remaining;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SWARM_DRONE_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_drone_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_swarm_drone_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_drone_data_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.gps_altitude , packet1.RTK_fix_type , packet1.battery_remaining , packet1.firmware_version , packet1.onboard_control_sensors_health , packet1.yaw );
    mavlink_msg_swarm_drone_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_drone_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.gps_altitude , packet1.RTK_fix_type , packet1.battery_remaining , packet1.firmware_version , packet1.onboard_control_sensors_health , packet1.yaw );
    mavlink_msg_swarm_drone_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_swarm_drone_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_drone_data_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude , packet1.gps_altitude , packet1.RTK_fix_type , packet1.battery_remaining , packet1.firmware_version , packet1.onboard_control_sensors_health , packet1.yaw );
    mavlink_msg_swarm_drone_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_position_target_wgs84_local_ned(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_POSITION_TARGET_WGS84_LOCAL_NED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_position_target_wgs84_local_ned_t packet_in = {
        963497464,963497672,963497880,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,19731,27,94,161
    };
    mavlink_set_position_target_wgs84_local_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.afx = packet_in.afx;
        packet1.afy = packet_in.afy;
        packet1.afz = packet_in.afz;
        packet1.yaw = packet_in.yaw;
        packet1.yaw_rate = packet_in.yaw_rate;
        packet1.type_mask = packet_in.type_mask;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.coordinate_frame = packet_in.coordinate_frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_POSITION_TARGET_WGS84_LOCAL_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_POSITION_TARGET_WGS84_LOCAL_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_wgs84_local_ned_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_position_target_wgs84_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_wgs84_local_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.latitude , packet1.longitude , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_wgs84_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_wgs84_local_ned_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.latitude , packet1.longitude , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_wgs84_local_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_position_target_wgs84_local_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_position_target_wgs84_local_ned_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.target_system , packet1.target_component , packet1.coordinate_frame , packet1.type_mask , packet1.latitude , packet1.longitude , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.afx , packet1.afy , packet1.afz , packet1.yaw , packet1.yaw_rate );
    mavlink_msg_set_position_target_wgs84_local_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hover(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_swarm_drone_data(system_id, component_id, last_msg);
    mavlink_test_set_position_target_wgs84_local_ned(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // HOVER_TESTSUITE_H
