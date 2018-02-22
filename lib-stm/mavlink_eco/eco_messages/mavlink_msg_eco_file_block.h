#pragma once
// MESSAGE ECO_FILE_BLOCK PACKING

#define MAVLINK_MSG_ID_ECO_FILE_BLOCK 203

MAVPACKED(
typedef struct __mavlink_eco_file_block_t {
 uint16_t log_id; /*< Log id [0..UINT16_MAX]*/
 uint16_t block_cnt; /*< Block count (log length) [0..UINT16_MAX]*/
 uint16_t block_seq; /*< Block sequence number [0..UINT16_MAX]*/
 uint8_t target_system; /*< Target system*/
 uint8_t block_len; /*< Block length [bytes] [0..128]*/
 uint8_t data[128]; /*< Data block*/
}) mavlink_eco_file_block_t;

#define MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN 136
#define MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN 136
#define MAVLINK_MSG_ID_203_LEN 136
#define MAVLINK_MSG_ID_203_MIN_LEN 136

#define MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC 237
#define MAVLINK_MSG_ID_203_CRC 237

#define MAVLINK_MSG_ECO_FILE_BLOCK_FIELD_DATA_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ECO_FILE_BLOCK { \
    203, \
    "ECO_FILE_BLOCK", \
    6, \
    {  { "log_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_eco_file_block_t, log_id) }, \
         { "block_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_eco_file_block_t, block_cnt) }, \
         { "block_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_eco_file_block_t, block_seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_eco_file_block_t, target_system) }, \
         { "block_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_eco_file_block_t, block_len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 8, offsetof(mavlink_eco_file_block_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ECO_FILE_BLOCK { \
    "ECO_FILE_BLOCK", \
    6, \
    {  { "log_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_eco_file_block_t, log_id) }, \
         { "block_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_eco_file_block_t, block_cnt) }, \
         { "block_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_eco_file_block_t, block_seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_eco_file_block_t, target_system) }, \
         { "block_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_eco_file_block_t, block_len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 128, 8, offsetof(mavlink_eco_file_block_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a eco_file_block message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]
 * @param block_cnt Block count (log length) [0..UINT16_MAX]
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param block_len Block length [bytes] [0..128]
 * @param data Data block
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_block_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, uint8_t block_len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, block_len);
    _mav_put_uint8_t_array(buf, 8, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN);
#else
    mavlink_eco_file_block_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    packet.block_len = block_len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_BLOCK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC);
}

/**
 * @brief Pack a eco_file_block message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]
 * @param block_cnt Block count (log length) [0..UINT16_MAX]
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param block_len Block length [bytes] [0..128]
 * @param data Data block
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_block_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint16_t log_id,uint16_t block_cnt,uint16_t block_seq,uint8_t block_len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, block_len);
    _mav_put_uint8_t_array(buf, 8, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN);
#else
    mavlink_eco_file_block_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    packet.block_len = block_len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_BLOCK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC);
}

/**
 * @brief Encode a eco_file_block struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_block C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_block_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eco_file_block_t* eco_file_block)
{
    return mavlink_msg_eco_file_block_pack(system_id, component_id, msg, eco_file_block->target_system, eco_file_block->log_id, eco_file_block->block_cnt, eco_file_block->block_seq, eco_file_block->block_len, eco_file_block->data);
}

/**
 * @brief Encode a eco_file_block struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_block C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_block_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eco_file_block_t* eco_file_block)
{
    return mavlink_msg_eco_file_block_pack_chan(system_id, component_id, chan, msg, eco_file_block->target_system, eco_file_block->log_id, eco_file_block->block_cnt, eco_file_block->block_seq, eco_file_block->block_len, eco_file_block->data);
}

/**
 * @brief Send a eco_file_block message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]
 * @param block_cnt Block count (log length) [0..UINT16_MAX]
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param block_len Block length [bytes] [0..128]
 * @param data Data block
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eco_file_block_send(mavlink_channel_t chan, uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, uint8_t block_len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, block_len);
    _mav_put_uint8_t_array(buf, 8, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_BLOCK, buf, MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC);
#else
    mavlink_eco_file_block_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    packet.block_len = block_len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_BLOCK, (const char *)&packet, MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC);
#endif
}

/**
 * @brief Send a eco_file_block message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eco_file_block_send_struct(mavlink_channel_t chan, const mavlink_eco_file_block_t* eco_file_block)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eco_file_block_send(chan, eco_file_block->target_system, eco_file_block->log_id, eco_file_block->block_cnt, eco_file_block->block_seq, eco_file_block->block_len, eco_file_block->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_BLOCK, (const char *)eco_file_block, MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC);
#endif
}

#if MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eco_file_block_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, uint8_t block_len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, block_len);
    _mav_put_uint8_t_array(buf, 8, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_BLOCK, buf, MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC);
#else
    mavlink_eco_file_block_t *packet = (mavlink_eco_file_block_t *)msgbuf;
    packet->log_id = log_id;
    packet->block_cnt = block_cnt;
    packet->block_seq = block_seq;
    packet->target_system = target_system;
    packet->block_len = block_len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_BLOCK, (const char *)packet, MAVLINK_MSG_ID_ECO_FILE_BLOCK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN, MAVLINK_MSG_ID_ECO_FILE_BLOCK_CRC);
#endif
}
#endif

#endif

// MESSAGE ECO_FILE_BLOCK UNPACKING


/**
 * @brief Get field target_system from eco_file_block message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_eco_file_block_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field log_id from eco_file_block message
 *
 * @return Log id [0..UINT16_MAX]
 */
static inline uint16_t mavlink_msg_eco_file_block_get_log_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field block_cnt from eco_file_block message
 *
 * @return Block count (log length) [0..UINT16_MAX]
 */
static inline uint16_t mavlink_msg_eco_file_block_get_block_cnt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field block_seq from eco_file_block message
 *
 * @return Block sequence number [0..UINT16_MAX]
 */
static inline uint16_t mavlink_msg_eco_file_block_get_block_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field block_len from eco_file_block message
 *
 * @return Block length [bytes] [0..128]
 */
static inline uint8_t mavlink_msg_eco_file_block_get_block_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field data from eco_file_block message
 *
 * @return Data block
 */
static inline uint16_t mavlink_msg_eco_file_block_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 128,  8);
}

/**
 * @brief Decode a eco_file_block message into a struct
 *
 * @param msg The message to decode
 * @param eco_file_block C-struct to decode the message contents into
 */
static inline void mavlink_msg_eco_file_block_decode(const mavlink_message_t* msg, mavlink_eco_file_block_t* eco_file_block)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eco_file_block->log_id = mavlink_msg_eco_file_block_get_log_id(msg);
    eco_file_block->block_cnt = mavlink_msg_eco_file_block_get_block_cnt(msg);
    eco_file_block->block_seq = mavlink_msg_eco_file_block_get_block_seq(msg);
    eco_file_block->target_system = mavlink_msg_eco_file_block_get_target_system(msg);
    eco_file_block->block_len = mavlink_msg_eco_file_block_get_block_len(msg);
    mavlink_msg_eco_file_block_get_data(msg, eco_file_block->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN? msg->len : MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN;
        memset(eco_file_block, 0, MAVLINK_MSG_ID_ECO_FILE_BLOCK_LEN);
    memcpy(eco_file_block, _MAV_PAYLOAD(msg), len);
#endif
}
