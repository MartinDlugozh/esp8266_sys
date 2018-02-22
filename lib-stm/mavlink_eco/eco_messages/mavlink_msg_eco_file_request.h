#pragma once
// MESSAGE ECO_FILE_REQUEST PACKING

#define MAVLINK_MSG_ID_ECO_FILE_REQUEST 200

MAVPACKED(
typedef struct __mavlink_eco_file_request_t {
 uint8_t target_system; /*< Target slave system  */
 uint8_t log_type; /*< Log type [continious/daily/all...]*/
 uint8_t op_type; /*< Operation type [download/delete/list]*/
}) mavlink_eco_file_request_t;

#define MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN 3
#define MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN 3
#define MAVLINK_MSG_ID_200_LEN 3
#define MAVLINK_MSG_ID_200_MIN_LEN 3

#define MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC 36
#define MAVLINK_MSG_ID_200_CRC 36



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ECO_FILE_REQUEST { \
    200, \
    "ECO_FILE_REQUEST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_eco_file_request_t, target_system) }, \
         { "log_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_eco_file_request_t, log_type) }, \
         { "op_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_eco_file_request_t, op_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ECO_FILE_REQUEST { \
    "ECO_FILE_REQUEST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_eco_file_request_t, target_system) }, \
         { "log_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_eco_file_request_t, log_type) }, \
         { "op_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_eco_file_request_t, op_type) }, \
         } \
}
#endif

/**
 * @brief Pack a eco_file_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system Target slave system  
 * @param log_type Log type [continious/daily/all...]
 * @param op_type Operation type [download/delete/list]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t log_type, uint8_t op_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, log_type);
    _mav_put_uint8_t(buf, 2, op_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN);
#else
    mavlink_eco_file_request_t packet;
    packet.target_system = target_system;
    packet.log_type = log_type;
    packet.op_type = op_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC);
}

/**
 * @brief Pack a eco_file_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system Target slave system  
 * @param log_type Log type [continious/daily/all...]
 * @param op_type Operation type [download/delete/list]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_file_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t log_type,uint8_t op_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, log_type);
    _mav_put_uint8_t(buf, 2, op_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN);
#else
    mavlink_eco_file_request_t packet;
    packet.target_system = target_system;
    packet.log_type = log_type;
    packet.op_type = op_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_FILE_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC);
}

/**
 * @brief Encode a eco_file_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eco_file_request_t* eco_file_request)
{
    return mavlink_msg_eco_file_request_pack(system_id, component_id, msg, eco_file_request->target_system, eco_file_request->log_type, eco_file_request->op_type);
}

/**
 * @brief Encode a eco_file_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eco_file_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_file_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eco_file_request_t* eco_file_request)
{
    return mavlink_msg_eco_file_request_pack_chan(system_id, component_id, chan, msg, eco_file_request->target_system, eco_file_request->log_type, eco_file_request->op_type);
}

/**
 * @brief Send a eco_file_request message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system Target slave system  
 * @param log_type Log type [continious/daily/all...]
 * @param op_type Operation type [download/delete/list]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eco_file_request_send(mavlink_channel_t chan, uint8_t target_system, uint8_t log_type, uint8_t op_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, log_type);
    _mav_put_uint8_t(buf, 2, op_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST, buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC);
#else
    mavlink_eco_file_request_t packet;
    packet.target_system = target_system;
    packet.log_type = log_type;
    packet.op_type = op_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC);
#endif
}

/**
 * @brief Send a eco_file_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eco_file_request_send_struct(mavlink_channel_t chan, const mavlink_eco_file_request_t* eco_file_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eco_file_request_send(chan, eco_file_request->target_system, eco_file_request->log_type, eco_file_request->op_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST, (const char *)eco_file_request, MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eco_file_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t log_type, uint8_t op_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, log_type);
    _mav_put_uint8_t(buf, 2, op_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST, buf, MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC);
#else
    mavlink_eco_file_request_t *packet = (mavlink_eco_file_request_t *)msgbuf;
    packet->target_system = target_system;
    packet->log_type = log_type;
    packet->op_type = op_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_FILE_REQUEST, (const char *)packet, MAVLINK_MSG_ID_ECO_FILE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN, MAVLINK_MSG_ID_ECO_FILE_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE ECO_FILE_REQUEST UNPACKING


/**
 * @brief Get field target_system from eco_file_request message
 *
 * @return Target slave system  
 */
static inline uint8_t mavlink_msg_eco_file_request_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field log_type from eco_file_request message
 *
 * @return Log type [continious/daily/all...]
 */
static inline uint8_t mavlink_msg_eco_file_request_get_log_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field op_type from eco_file_request message
 *
 * @return Operation type [download/delete/list]
 */
static inline uint8_t mavlink_msg_eco_file_request_get_op_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a eco_file_request message into a struct
 *
 * @param msg The message to decode
 * @param eco_file_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_eco_file_request_decode(const mavlink_message_t* msg, mavlink_eco_file_request_t* eco_file_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eco_file_request->target_system = mavlink_msg_eco_file_request_get_target_system(msg);
    eco_file_request->log_type = mavlink_msg_eco_file_request_get_log_type(msg);
    eco_file_request->op_type = mavlink_msg_eco_file_request_get_op_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN;
        memset(eco_file_request, 0, MAVLINK_MSG_ID_ECO_FILE_REQUEST_LEN);
    memcpy(eco_file_request, _MAV_PAYLOAD(msg), len);
#endif
}
