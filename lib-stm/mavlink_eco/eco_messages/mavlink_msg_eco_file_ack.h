#pragma once
// MESSAGE ECO_FILE_ACK PACKING

#define MAVLINK_MSG_ID_ECO_FILE_ACK 202

MAVPACKED(
typedef struct __mavlink_eco_file_ack_t {
 uint16_t log_id; /*< Log id [0..UINT16_MAX]*/
 uint16_t block_cnt; /*< Block count (log length) [0..UINT16_MAX]*/
 uint16_t block_seq; /*< Block sequence number [0..UINT16_MAX]*/
 uint8_t target_system; /*< Target system*/
 uint8_t ack; /*< Acknowledge. 1 - continue transfer starting from the next block; 0 - continue transfer starting from the current block*/
}) mavlink_eco_file_ack_t;

#define MAVLINK_MSG_ID_ECO_FILE_ACK_LEN 8
#define MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN 8
#define MAVLINK_MSG_ID_202_LEN 8
#define MAVLINK_MSG_ID_202_MIN_LEN 8

#define MAVLINK_MSG_ID_ECO_FILE_ACK_CRC 23
#define MAVLINK_MSG_ID_202_CRC 23



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ECO_FILE_ACK { \
    202, \
    "ECO_FILE_ACK", \
    5, \
    {  { "log_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_eco_file_ack_t, log_id) }, \
         { "block_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_eco_file_ack_t, block_cnt) }, \
         { "block_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_eco_file_ack_t, block_seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_eco_file_ack_t, target_system) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_eco_file_ack_t, ack) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ECO_FILE_ACK { \
    "ECO_FILE_ACK", \
    5, \
    {  { "log_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_eco_file_ack_t, log_id) }, \
         { "block_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_eco_file_ack_t, block_cnt) }, \
         { "block_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_eco_file_ack_t, block_seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_eco_file_ack_t, target_system) }, \
         { "ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_eco_file_ack_t, ack) }, \
         } \
}
#endif

/**
 * @brief Pack a eco_file_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]
 * @param block_cnt Block count (log length) [0..UINT16_MAX]
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param ack Acknowledge. 1 - continue transfer starting from the next block; 0 - continue transfer starting from the current block
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, uint8_t ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_ACK_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, ack);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN);
#else
    mavlink_eco_file_ack_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    packet.ack = ack;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_CRC);
}

/**
 * @brief Pack a eco_file_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]
 * @param block_cnt Block count (log length) [0..UINT16_MAX]
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param ack Acknowledge. 1 - continue transfer starting from the next block; 0 - continue transfer starting from the current block
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint16_t log_id,uint16_t block_cnt,uint16_t block_seq,uint8_t ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_ACK_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, ack);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN);
#else
    mavlink_eco_file_ack_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    packet.ack = ack;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_CRC);
}

/**
 * @brief Encode a eco_file_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eco_file_ack_t* eco_file_ack)
{
    return mavlink_msg_eco_file_ack_pack(system_id, component_id, msg, eco_file_ack->target_system, eco_file_ack->log_id, eco_file_ack->block_cnt, eco_file_ack->block_seq, eco_file_ack->ack);
}

/**
 * @brief Encode a eco_file_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eco_file_ack_t* eco_file_ack)
{
    return mavlink_msg_eco_file_ack_pack_chan(system_id, component_id, chan, msg, eco_file_ack->target_system, eco_file_ack->log_id, eco_file_ack->block_cnt, eco_file_ack->block_seq, eco_file_ack->ack);
}

/**
 * @brief Send a eco_file_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system Target system
 * @param log_id Log id [0..UINT16_MAX]
 * @param block_cnt Block count (log length) [0..UINT16_MAX]
 * @param block_seq Block sequence number [0..UINT16_MAX]
 * @param ack Acknowledge. 1 - continue transfer starting from the next block; 0 - continue transfer starting from the current block
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eco_file_ack_send(mavlink_channel_t chan, uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, uint8_t ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_ACK_LEN];
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, ack);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_ACK, buf, MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_CRC);
#else
    mavlink_eco_file_ack_t packet;
    packet.log_id = log_id;
    packet.block_cnt = block_cnt;
    packet.block_seq = block_seq;
    packet.target_system = target_system;
    packet.ack = ack;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_ACK, (const char *)&packet, MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_CRC);
#endif
}

/**
 * @brief Send a eco_file_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eco_file_ack_send_struct(mavlink_channel_t chan, const mavlink_eco_file_ack_t* eco_file_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eco_file_ack_send(chan, eco_file_ack->target_system, eco_file_ack->log_id, eco_file_ack->block_cnt, eco_file_ack->block_seq, eco_file_ack->ack);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_ACK, (const char *)eco_file_ack, MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_ECO_FILE_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eco_file_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint16_t log_id, uint16_t block_cnt, uint16_t block_seq, uint8_t ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, log_id);
    _mav_put_uint16_t(buf, 2, block_cnt);
    _mav_put_uint16_t(buf, 4, block_seq);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, ack);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_ACK, buf, MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_CRC);
#else
    mavlink_eco_file_ack_t *packet = (mavlink_eco_file_ack_t *)msgbuf;
    packet->log_id = log_id;
    packet->block_cnt = block_cnt;
    packet->block_seq = block_seq;
    packet->target_system = target_system;
    packet->ack = ack;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_ACK, (const char *)packet, MAVLINK_MSG_ID_ECO_FILE_ACK_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN, MAVLINK_MSG_ID_ECO_FILE_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE ECO_FILE_ACK UNPACKING


/**
 * @brief Get field target_system from eco_file_ack message
 *
 * @return Target system
 */
static inline uint8_t mavlink_msg_eco_file_ack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field log_id from eco_file_ack message
 *
 * @return Log id [0..UINT16_MAX]
 */
static inline uint16_t mavlink_msg_eco_file_ack_get_log_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field block_cnt from eco_file_ack message
 *
 * @return Block count (log length) [0..UINT16_MAX]
 */
static inline uint16_t mavlink_msg_eco_file_ack_get_block_cnt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field block_seq from eco_file_ack message
 *
 * @return Block sequence number [0..UINT16_MAX]
 */
static inline uint16_t mavlink_msg_eco_file_ack_get_block_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field ack from eco_file_ack message
 *
 * @return Acknowledge. 1 - continue transfer starting from the next block; 0 - continue transfer starting from the current block
 */
static inline uint8_t mavlink_msg_eco_file_ack_get_ack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a eco_file_ack message into a struct
 *
 * @param msg The message to decode
 * @param eco_file_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_eco_file_ack_decode(const mavlink_message_t* msg, mavlink_eco_file_ack_t* eco_file_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eco_file_ack->log_id = mavlink_msg_eco_file_ack_get_log_id(msg);
    eco_file_ack->block_cnt = mavlink_msg_eco_file_ack_get_block_cnt(msg);
    eco_file_ack->block_seq = mavlink_msg_eco_file_ack_get_block_seq(msg);
    eco_file_ack->target_system = mavlink_msg_eco_file_ack_get_target_system(msg);
    eco_file_ack->ack = mavlink_msg_eco_file_ack_get_ack(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ECO_FILE_ACK_LEN? msg->len : MAVLINK_MSG_ID_ECO_FILE_ACK_LEN;
        memset(eco_file_ack, 0, MAVLINK_MSG_ID_ECO_FILE_ACK_LEN);
    memcpy(eco_file_ack, _MAV_PAYLOAD(msg), len);
#endif
}
