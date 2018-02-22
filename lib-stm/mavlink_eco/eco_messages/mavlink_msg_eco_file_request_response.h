#pragma once
// MESSAGE ECO_FILE_REQUEST_RESPONSE PACKING

#define MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE 201

MAVPACKED(
typedef struct __mavlink_eco_file_request_response_t {
 uint16_t log_id; /*< Log id [0..UINT16_MAX]. If zero - no files present*/
 uint16_t block_cnt; /*< Block count (log length) [0..UINT16_MAX]. If zero - log file is empty or no files present*/
 uint16_t block_seq; /*< Block sequence number [0..UINT16_MAX]*/
 uint8_t target_system; /*< Target system*/
 uint8_t log_name[128]; /*< Log file name*/
}) mavlink_eco_file_request_response_t;

#define MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN 135
#define MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN 135
#define MAVLINK_MSG_ID_201_LEN 135
#define MAVLINK_MSG_ID_201_MIN_LEN 135

#define MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC 134
#define MAVLINK_MSG_ID_201_CRC 134

#define MAVLINK_MSG_ECO_FILE_REQUEST_RESPONSE_FIELD_LOG_NAME_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ECO_FILE_REQUEST_RESPONSE { \
    201, \
    "ECO_FILE_REQUEST_RESPONSE", \
    5, \
    {  { "log_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_eco_file_request_response_t, log_id) }, \
         { "block_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_eco_file_request_response_t, block_cnt) }, \
         { "block_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_eco_file_request_response_t, block_seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_eco_file_request_response_t, target_system) }, \
         { "log_name", NULL, MAVLINK_TYPE_UINT8_T, 128, 7, offsetof(mavlink_eco_file_request_response_t, log_name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ECO_FILE_REQUEST_RESPONSE { \
    "ECO_FILE_REQUEST_RESPONSE", \
    5, \
    {  { "log_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_eco_file_request_response_t, log_id) }, \
         { "block_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_eco_file_request_response_t, block_cnt) }, \
         { "block_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_eco_file_request_response_t, block_seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_eco_file_request_response_t, target_system) }, \
         { "log_name", NULL, MAVLINK_TYPE_UINT8_T, 128, 7, offsetof(mavlink_eco_file_request_response_t, log_name) }, \
         } \
}
#endif

/**
 * @brief Pack a eco_file_request_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]. If zero - no files present
 * @param block_cnt Block count (log length) [0..UINT16_MAX]. If zero - log file is empty or no files present
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param log_name Log file name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_request_response_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, const uint8_t *log_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t_array(buf, 7, log_name, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN);
#else
    mavlink_eco_file_request_response_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    mav_array_memcpy(packet.log_name, log_name, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC);
}

/**
 * @brief Pack a eco_file_request_response message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]. If zero - no files present
 * @param block_cnt Block count (log length) [0..UINT16_MAX]. If zero - log file is empty or no files present
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param log_name Log file name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_request_response_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint16_t log_id,uint16_t block_cnt,uint16_t block_seq,const uint8_t *log_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t_array(buf, 7, log_name, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN);
#else
    mavlink_eco_file_request_response_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    mav_array_memcpy(packet.log_name, log_name, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC);
}

/**
 * @brief Encode a eco_file_request_response struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_request_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_request_response_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eco_file_request_response_t* eco_file_request_response)
{
    return mavlink_msg_eco_file_request_response_pack(system_id, component_id, msg, eco_file_request_response->target_system, eco_file_request_response->log_id, eco_file_request_response->block_cnt, eco_file_request_response->block_seq, eco_file_request_response->log_name);
}

/**
 * @brief Encode a eco_file_request_response struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_request_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_request_response_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eco_file_request_response_t* eco_file_request_response)
{
    return mavlink_msg_eco_file_request_response_pack_chan(system_id, component_id, chan, msg, eco_file_request_response->target_system, eco_file_request_response->log_id, eco_file_request_response->block_cnt, eco_file_request_response->block_seq, eco_file_request_response->log_name);
}

/**
 * @brief Send a eco_file_request_response message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]. If zero - no files present
 * @param block_cnt Block count (log length) [0..UINT16_MAX]. If zero - log file is empty or no files present
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param log_name Log file name
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eco_file_request_response_send(mavlink_channel_t chan, uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, const uint8_t *log_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t_array(buf, 7, log_name, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE, buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC);
#else
    mavlink_eco_file_request_response_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    mav_array_memcpy(packet.log_name, log_name, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE, (const char *)&packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC);
#endif
}

/**
 * @brief Send a eco_file_request_response message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eco_file_request_response_send_struct(mavlink_channel_t chan, const mavlink_eco_file_request_response_t* eco_file_request_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eco_file_request_response_send(chan, eco_file_request_response->target_system, eco_file_request_response->log_id, eco_file_request_response->block_cnt, eco_file_request_response->block_seq, eco_file_request_response->log_name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE, (const char *)eco_file_request_response, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eco_file_request_response_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, const uint8_t *log_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t_array(buf, 7, log_name, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE, buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC);
#else
    mavlink_eco_file_request_response_t *packet = (mavlink_eco_file_request_response_t *)msgbuf;
    packet->log_id = log_id;
    packet->block_cnt = block_cnt;
    packet->block_seq = block_seq;
    packet->target_system = target_system;
    mav_array_memcpy(packet->log_name, log_name, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE, (const char *)packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_CRC);
#endif
}
#endif

#endif

// MESSAGE ECO_FILE_REQUEST_RESPONSE UNPACKING


/**
 * @brief Get field target_system from eco_file_request_response message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_eco_file_request_response_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field log_id from eco_file_request_response message
 *
 * @return Log id [0..UINT16_MAX]. If zero - no files present
 */
static inline uint16_t mavlink_msg_eco_file_request_response_get_log_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field block_cnt from eco_file_request_response message
 *
 * @return Block count (log length) [0..UINT16_MAX]. If zero - log file is empty or no files present
 */
static inline uint16_t mavlink_msg_eco_file_request_response_get_block_cnt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field block_seq from eco_file_request_response message
 *
 * @return Block sequence number [0..UINT16_MAX]
 */
static inline uint16_t mavlink_msg_eco_file_request_response_get_block_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field log_name from eco_file_request_response message
 *
 * @return Log file name
 */
static inline uint16_t mavlink_msg_eco_file_request_response_get_log_name(const mavlink_message_t* msg, uint8_t *log_name)
{
    return _MAV_RETURN_uint8_t_array(msg, log_name, 128,  7);
}

/**
 * @brief Decode a eco_file_request_response message into a struct
 *
 * @param msg The message to decode
 * @param eco_file_request_response C-struct to decode the message contents into
 */
static inline void mavlink_msg_eco_file_request_response_decode(const mavlink_message_t* msg, mavlink_eco_file_request_response_t* eco_file_request_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eco_file_request_response->log_id = mavlink_msg_eco_file_request_response_get_log_id(msg);
    eco_file_request_response->block_cnt = mavlink_msg_eco_file_request_response_get_block_cnt(msg);
    eco_file_request_response->block_seq = mavlink_msg_eco_file_request_response_get_block_seq(msg);
    eco_file_request_response->target_system = mavlink_msg_eco_file_request_response_get_target_system(msg);
    mavlink_msg_eco_file_request_response_get_log_name(msg, eco_file_request_response->log_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN? msg->len : MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN;
        memset(eco_file_request_response, 0, MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE_LEN);
    memcpy(eco_file_request_response, _MAV_PAYLOAD(msg), len);
#endif
}
