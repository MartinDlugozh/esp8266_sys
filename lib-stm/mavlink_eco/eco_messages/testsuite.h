/** @file
 *    @brief MAVLink comm protocol testsuite generated from eco_messages.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef ECO_MESSAGES_TESTSUITE_H
#define ECO_MESSAGES_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_eco_messages(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_eco_messages(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_eco_sys_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_SYS_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_sys_status_t packet_in = {
        17.0,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93 }
    };
    mavlink_eco_sys_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.votage = packet_in.votage;
        packet1.wifi_connection = packet_in.wifi_connection;
        
        mav_array_memcpy(packet1.sensors, packet_in.sensors, sizeof(uint8_t)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_SYS_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_SYS_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sys_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sys_status_pack(system_id, component_id, &msg , packet1.wifi_connection , packet1.sensors , packet1.votage );
    mavlink_msg_eco_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sys_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.wifi_connection , packet1.sensors , packet1.votage );
    mavlink_msg_eco_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sys_status_send(MAVLINK_COMM_1 , packet1.wifi_connection , packet1.sensors , packet1.votage );
    mavlink_msg_eco_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_bmp180(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_BMP180 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_bmp180_t packet_in = {
        17.0,45.0,963497880
    };
    mavlink_eco_bmp180_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.temperature = packet_in.temperature;
        packet1.altitude = packet_in.altitude;
        packet1.presssure = packet_in.presssure;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_bmp180_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_bmp180_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_bmp180_pack(system_id, component_id, &msg , packet1.temperature , packet1.altitude , packet1.presssure );
    mavlink_msg_eco_bmp180_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_bmp180_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.altitude , packet1.presssure );
    mavlink_msg_eco_bmp180_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_bmp180_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_bmp180_send(MAVLINK_COMM_1 , packet1.temperature , packet1.altitude , packet1.presssure );
    mavlink_msg_eco_bmp180_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_max44009(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_MAX44009 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_max44009_t packet_in = {
        963497464,963497672
    };
    mavlink_eco_max44009_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lux_ambilight_1 = packet_in.lux_ambilight_1;
        packet1.lux_ambilight_2 = packet_in.lux_ambilight_2;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_max44009_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_max44009_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_max44009_pack(system_id, component_id, &msg , packet1.lux_ambilight_1 , packet1.lux_ambilight_2 );
    mavlink_msg_eco_max44009_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_max44009_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lux_ambilight_1 , packet1.lux_ambilight_2 );
    mavlink_msg_eco_max44009_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_max44009_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_max44009_send(MAVLINK_COMM_1 , packet1.lux_ambilight_1 , packet1.lux_ambilight_2 );
    mavlink_msg_eco_max44009_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_sht11(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_SHT11 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_sht11_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_eco_sht11_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.temperature = packet_in.temperature;
        packet1.humidity = packet_in.humidity;
        packet1.dowpoint = packet_in.dowpoint;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sht11_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_sht11_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sht11_pack(system_id, component_id, &msg , packet1.temperature , packet1.humidity , packet1.dowpoint );
    mavlink_msg_eco_sht11_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sht11_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.humidity , packet1.dowpoint );
    mavlink_msg_eco_sht11_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_sht11_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_sht11_send(MAVLINK_COMM_1 , packet1.temperature , packet1.humidity , packet1.dowpoint );
    mavlink_msg_eco_sht11_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_file_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_FILE_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_file_request_t packet_in = {
        17235,139,206,17
    };
    mavlink_eco_file_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.log_id = packet_in.log_id;
        packet1.target_system = packet_in.target_system;
        packet1.log_type = packet_in.log_type;
        packet1.op_type = packet_in.op_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_file_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.log_id , packet1.log_type , packet1.op_type );
    mavlink_msg_eco_file_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.log_id , packet1.log_type , packet1.op_type );
    mavlink_msg_eco_file_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_file_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.log_id , packet1.log_type , packet1.op_type );
    mavlink_msg_eco_file_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_file_request_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_file_request_response_t packet_in = {
        17235,17339,17443,151,{ 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89 }
    };
    mavlink_eco_file_request_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.log_id = packet_in.log_id;
        packet1.block_cnt = packet_in.block_cnt;
        packet1.block_seq = packet_in.block_seq;
        packet1.target_system = packet_in.target_system;
        
        mav_array_memcpy(packet1.log_name, packet_in.log_name, sizeof(uint8_t)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_file_request_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_response_pack(system_id, component_id, &msg , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.log_name );
    mavlink_msg_eco_file_request_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.log_name );
    mavlink_msg_eco_file_request_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_file_request_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_request_response_send(MAVLINK_COMM_1 , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.log_name );
    mavlink_msg_eco_file_request_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_file_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_FILE_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_file_ack_t packet_in = {
        17235,17339,17443,151,218
    };
    mavlink_eco_file_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.log_id = packet_in.log_id;
        packet1.block_cnt = packet_in.block_cnt;
        packet1.block_seq = packet_in.block_seq;
        packet1.target_system = packet_in.target_system;
        packet1.ack = packet_in.ack;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_file_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_ack_pack(system_id, component_id, &msg , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.ack );
    mavlink_msg_eco_file_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.ack );
    mavlink_msg_eco_file_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_file_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_ack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.ack );
    mavlink_msg_eco_file_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_file_block(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ECO_FILE_BLOCK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_eco_file_block_t packet_in = {
        17235,17339,17443,151,218,{ 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156 }
    };
    mavlink_eco_file_block_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.log_id = packet_in.log_id;
        packet1.block_cnt = packet_in.block_cnt;
        packet1.block_seq = packet_in.block_seq;
        packet1.target_system = packet_in.target_system;
        packet1.block_len = packet_in.block_len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_block_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_eco_file_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_block_pack(system_id, component_id, &msg , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.block_len , packet1.data );
    mavlink_msg_eco_file_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_block_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.block_len , packet1.data );
    mavlink_msg_eco_file_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_eco_file_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_eco_file_block_send(MAVLINK_COMM_1 , packet1.target_system , packet1.log_id , packet1.block_cnt , packet1.block_seq , packet1.block_len , packet1.data );
    mavlink_msg_eco_file_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_eco_messages(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_eco_sys_status(system_id, component_id, last_msg);
    mavlink_test_eco_bmp180(system_id, component_id, last_msg);
    mavlink_test_eco_max44009(system_id, component_id, last_msg);
    mavlink_test_eco_sht11(system_id, component_id, last_msg);
    mavlink_test_eco_file_request(system_id, component_id, last_msg);
    mavlink_test_eco_file_request_response(system_id, component_id, last_msg);
    mavlink_test_eco_file_ack(system_id, component_id, last_msg);
    mavlink_test_eco_file_block(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ECO_MESSAGES_TESTSUITE_H
