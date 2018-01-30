#pragma once
// MESSAGE ECO_MAX44009 PACKING

#define MAVLINK_MSG_ID_ECO_MAX44009 192

MAVPACKED(
typedef struct __mavlink_eco_max44009_t {
 uint32_t lux_ambilight_1; /*< Ambient light from first sensor [lx]*/
 uint32_t lux_ambilight_2; /*< Ambient light from second sensor [lx]*/
}) mavlink_eco_max44009_t;

#define MAVLINK_MSG_ID_ECO_MAX44009_LEN 8
#define MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN 8
#define MAVLINK_MSG_ID_192_LEN 8
#define MAVLINK_MSG_ID_192_MIN_LEN 8

#define MAVLINK_MSG_ID_ECO_MAX44009_CRC 103
#define MAVLINK_MSG_ID_192_CRC 103



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ECO_MAX44009 { \
    192, \
    "ECO_MAX44009", \
    2, \
    {  { "lux_ambilight_1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_eco_max44009_t, lux_ambilight_1) }, \
         { "lux_ambilight_2", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_eco_max44009_t, lux_ambilight_2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ECO_MAX44009 { \
    "ECO_MAX44009", \
    2, \
    {  { "lux_ambilight_1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_eco_max44009_t, lux_ambilight_1) }, \
         { "lux_ambilight_2", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_eco_max44009_t, lux_ambilight_2) }, \
         } \
}
#endif

/**
 * @brief Pack a eco_max44009 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lux_ambilight_1 Ambient light from first sensor [lx]
 * @param lux_ambilight_2 Ambient light from second sensor [lx]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_max44009_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t lux_ambilight_1, uint32_t lux_ambilight_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_MAX44009_LEN];
    _mav_put_uint32_t(buf, 0, lux_ambilight_1);
    _mav_put_uint32_t(buf, 4, lux_ambilight_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_MAX44009_LEN);
#else
    mavlink_eco_max44009_t packet;
    packet.lux_ambilight_1 = lux_ambilight_1;
    packet.lux_ambilight_2 = lux_ambilight_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_MAX44009_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_MAX44009;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN, MAVLINK_MSG_ID_ECO_MAX44009_LEN, MAVLINK_MSG_ID_ECO_MAX44009_CRC);
}

/**
 * @brief Pack a eco_max44009 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lux_ambilight_1 Ambient light from first sensor [lx]
 * @param lux_ambilight_2 Ambient light from second sensor [lx]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_max44009_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t lux_ambilight_1,uint32_t lux_ambilight_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_MAX44009_LEN];
    _mav_put_uint32_t(buf, 0, lux_ambilight_1);
    _mav_put_uint32_t(buf, 4, lux_ambilight_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_MAX44009_LEN);
#else
    mavlink_eco_max44009_t packet;
    packet.lux_ambilight_1 = lux_ambilight_1;
    packet.lux_ambilight_2 = lux_ambilight_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_MAX44009_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_MAX44009;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN, MAVLINK_MSG_ID_ECO_MAX44009_LEN, MAVLINK_MSG_ID_ECO_MAX44009_CRC);
}

/**
 * @brief Encode a eco_max44009 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eco_max44009 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_max44009_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eco_max44009_t* eco_max44009)
{
    return mavlink_msg_eco_max44009_pack(system_id, component_id, msg, eco_max44009->lux_ambilight_1, eco_max44009->lux_ambilight_2);
}

/**
 * @brief Encode a eco_max44009 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eco_max44009 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_max44009_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eco_max44009_t* eco_max44009)
{
    return mavlink_msg_eco_max44009_pack_chan(system_id, component_id, chan, msg, eco_max44009->lux_ambilight_1, eco_max44009->lux_ambilight_2);
}

/**
 * @brief Send a eco_max44009 message
 * @param chan MAVLink channel to send the message
 *
 * @param lux_ambilight_1 Ambient light from first sensor [lx]
 * @param lux_ambilight_2 Ambient light from second sensor [lx]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eco_max44009_send(mavlink_channel_t chan, uint32_t lux_ambilight_1, uint32_t lux_ambilight_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_MAX44009_LEN];
    _mav_put_uint32_t(buf, 0, lux_ambilight_1);
    _mav_put_uint32_t(buf, 4, lux_ambilight_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_MAX44009, buf, MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN, MAVLINK_MSG_ID_ECO_MAX44009_LEN, MAVLINK_MSG_ID_ECO_MAX44009_CRC);
#else
    mavlink_eco_max44009_t packet;
    packet.lux_ambilight_1 = lux_ambilight_1;
    packet.lux_ambilight_2 = lux_ambilight_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_MAX44009, (const char *)&packet, MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN, MAVLINK_MSG_ID_ECO_MAX44009_LEN, MAVLINK_MSG_ID_ECO_MAX44009_CRC);
#endif
}

/**
 * @brief Send a eco_max44009 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eco_max44009_send_struct(mavlink_channel_t chan, const mavlink_eco_max44009_t* eco_max44009)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eco_max44009_send(chan, eco_max44009->lux_ambilight_1, eco_max44009->lux_ambilight_2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_MAX44009, (const char *)eco_max44009, MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN, MAVLINK_MSG_ID_ECO_MAX44009_LEN, MAVLINK_MSG_ID_ECO_MAX44009_CRC);
#endif
}

#if MAVLINK_MSG_ID_ECO_MAX44009_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eco_max44009_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t lux_ambilight_1, uint32_t lux_ambilight_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, lux_ambilight_1);
    _mav_put_uint32_t(buf, 4, lux_ambilight_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_MAX44009, buf, MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN, MAVLINK_MSG_ID_ECO_MAX44009_LEN, MAVLINK_MSG_ID_ECO_MAX44009_CRC);
#else
    mavlink_eco_max44009_t *packet = (mavlink_eco_max44009_t *)msgbuf;
    packet->lux_ambilight_1 = lux_ambilight_1;
    packet->lux_ambilight_2 = lux_ambilight_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_MAX44009, (const char *)packet, MAVLINK_MSG_ID_ECO_MAX44009_MIN_LEN, MAVLINK_MSG_ID_ECO_MAX44009_LEN, MAVLINK_MSG_ID_ECO_MAX44009_CRC);
#endif
}
#endif

#endif

// MESSAGE ECO_MAX44009 UNPACKING


/**
 * @brief Get field lux_ambilight_1 from eco_max44009 message
 *
 * @return Ambient light from first sensor [lx]
 */
static inline uint32_t mavlink_msg_eco_max44009_get_lux_ambilight_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lux_ambilight_2 from eco_max44009 message
 *
 * @return Ambient light from second sensor [lx]
 */
static inline uint32_t mavlink_msg_eco_max44009_get_lux_ambilight_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a eco_max44009 message into a struct
 *
 * @param msg The message to decode
 * @param eco_max44009 C-struct to decode the message contents into
 */
static inline void mavlink_msg_eco_max44009_decode(const mavlink_message_t* msg, mavlink_eco_max44009_t* eco_max44009)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eco_max44009->lux_ambilight_1 = mavlink_msg_eco_max44009_get_lux_ambilight_1(msg);
    eco_max44009->lux_ambilight_2 = mavlink_msg_eco_max44009_get_lux_ambilight_2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ECO_MAX44009_LEN? msg->len : MAVLINK_MSG_ID_ECO_MAX44009_LEN;
        memset(eco_max44009, 0, MAVLINK_MSG_ID_ECO_MAX44009_LEN);
    memcpy(eco_max44009, _MAV_PAYLOAD(msg), len);
#endif
}
