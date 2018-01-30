#pragma once
// MESSAGE ECO_BMP180 PACKING

#define MAVLINK_MSG_ID_ECO_BMP180 191

MAVPACKED(
typedef struct __mavlink_eco_bmp180_t {
 float temperature; /*< Temperature from BMP180 sensor [degC]*/
 float altitude; /*< Calculated barometric altitude (abs) [m]*/
 uint32_t presssure; /*< Pressure from BMP180 sensor[pa]*/
}) mavlink_eco_bmp180_t;

#define MAVLINK_MSG_ID_ECO_BMP180_LEN 12
#define MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN 12
#define MAVLINK_MSG_ID_191_LEN 12
#define MAVLINK_MSG_ID_191_MIN_LEN 12

#define MAVLINK_MSG_ID_ECO_BMP180_CRC 32
#define MAVLINK_MSG_ID_191_CRC 32



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ECO_BMP180 { \
    191, \
    "ECO_BMP180", \
    3, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_eco_bmp180_t, temperature) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_eco_bmp180_t, altitude) }, \
         { "presssure", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_eco_bmp180_t, presssure) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ECO_BMP180 { \
    "ECO_BMP180", \
    3, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_eco_bmp180_t, temperature) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_eco_bmp180_t, altitude) }, \
         { "presssure", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_eco_bmp180_t, presssure) }, \
         } \
}
#endif

/**
 * @brief Pack a eco_bmp180 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature Temperature from BMP180 sensor [degC]
 * @param altitude Calculated barometric altitude (abs) [m]
 * @param presssure Pressure from BMP180 sensor[pa]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_bmp180_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float temperature, float altitude, uint32_t presssure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_BMP180_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, altitude);
    _mav_put_uint32_t(buf, 8, presssure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_BMP180_LEN);
#else
    mavlink_eco_bmp180_t packet;
    packet.temperature = temperature;
    packet.altitude = altitude;
    packet.presssure = presssure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_BMP180_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_BMP180;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN, MAVLINK_MSG_ID_ECO_BMP180_LEN, MAVLINK_MSG_ID_ECO_BMP180_CRC);
}

/**
 * @brief Pack a eco_bmp180 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature Temperature from BMP180 sensor [degC]
 * @param altitude Calculated barometric altitude (abs) [m]
 * @param presssure Pressure from BMP180 sensor[pa]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_bmp180_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float temperature,float altitude,uint32_t presssure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_BMP180_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, altitude);
    _mav_put_uint32_t(buf, 8, presssure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_BMP180_LEN);
#else
    mavlink_eco_bmp180_t packet;
    packet.temperature = temperature;
    packet.altitude = altitude;
    packet.presssure = presssure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_BMP180_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_BMP180;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN, MAVLINK_MSG_ID_ECO_BMP180_LEN, MAVLINK_MSG_ID_ECO_BMP180_CRC);
}

/**
 * @brief Encode a eco_bmp180 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eco_bmp180 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_bmp180_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eco_bmp180_t* eco_bmp180)
{
    return mavlink_msg_eco_bmp180_pack(system_id, component_id, msg, eco_bmp180->temperature, eco_bmp180->altitude, eco_bmp180->presssure);
}

/**
 * @brief Encode a eco_bmp180 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eco_bmp180 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_bmp180_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eco_bmp180_t* eco_bmp180)
{
    return mavlink_msg_eco_bmp180_pack_chan(system_id, component_id, chan, msg, eco_bmp180->temperature, eco_bmp180->altitude, eco_bmp180->presssure);
}

/**
 * @brief Send a eco_bmp180 message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature Temperature from BMP180 sensor [degC]
 * @param altitude Calculated barometric altitude (abs) [m]
 * @param presssure Pressure from BMP180 sensor[pa]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eco_bmp180_send(mavlink_channel_t chan, float temperature, float altitude, uint32_t presssure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_BMP180_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, altitude);
    _mav_put_uint32_t(buf, 8, presssure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_BMP180, buf, MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN, MAVLINK_MSG_ID_ECO_BMP180_LEN, MAVLINK_MSG_ID_ECO_BMP180_CRC);
#else
    mavlink_eco_bmp180_t packet;
    packet.temperature = temperature;
    packet.altitude = altitude;
    packet.presssure = presssure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_BMP180, (const char *)&packet, MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN, MAVLINK_MSG_ID_ECO_BMP180_LEN, MAVLINK_MSG_ID_ECO_BMP180_CRC);
#endif
}

/**
 * @brief Send a eco_bmp180 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eco_bmp180_send_struct(mavlink_channel_t chan, const mavlink_eco_bmp180_t* eco_bmp180)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eco_bmp180_send(chan, eco_bmp180->temperature, eco_bmp180->altitude, eco_bmp180->presssure);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_BMP180, (const char *)eco_bmp180, MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN, MAVLINK_MSG_ID_ECO_BMP180_LEN, MAVLINK_MSG_ID_ECO_BMP180_CRC);
#endif
}

#if MAVLINK_MSG_ID_ECO_BMP180_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eco_bmp180_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float temperature, float altitude, uint32_t presssure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, altitude);
    _mav_put_uint32_t(buf, 8, presssure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_BMP180, buf, MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN, MAVLINK_MSG_ID_ECO_BMP180_LEN, MAVLINK_MSG_ID_ECO_BMP180_CRC);
#else
    mavlink_eco_bmp180_t *packet = (mavlink_eco_bmp180_t *)msgbuf;
    packet->temperature = temperature;
    packet->altitude = altitude;
    packet->presssure = presssure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_BMP180, (const char *)packet, MAVLINK_MSG_ID_ECO_BMP180_MIN_LEN, MAVLINK_MSG_ID_ECO_BMP180_LEN, MAVLINK_MSG_ID_ECO_BMP180_CRC);
#endif
}
#endif

#endif

// MESSAGE ECO_BMP180 UNPACKING


/**
 * @brief Get field temperature from eco_bmp180 message
 *
 * @return Temperature from BMP180 sensor [degC]
 */
static inline float mavlink_msg_eco_bmp180_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field altitude from eco_bmp180 message
 *
 * @return Calculated barometric altitude (abs) [m]
 */
static inline float mavlink_msg_eco_bmp180_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field presssure from eco_bmp180 message
 *
 * @return Pressure from BMP180 sensor[pa]
 */
static inline uint32_t mavlink_msg_eco_bmp180_get_presssure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a eco_bmp180 message into a struct
 *
 * @param msg The message to decode
 * @param eco_bmp180 C-struct to decode the message contents into
 */
static inline void mavlink_msg_eco_bmp180_decode(const mavlink_message_t* msg, mavlink_eco_bmp180_t* eco_bmp180)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eco_bmp180->temperature = mavlink_msg_eco_bmp180_get_temperature(msg);
    eco_bmp180->altitude = mavlink_msg_eco_bmp180_get_altitude(msg);
    eco_bmp180->presssure = mavlink_msg_eco_bmp180_get_presssure(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ECO_BMP180_LEN? msg->len : MAVLINK_MSG_ID_ECO_BMP180_LEN;
        memset(eco_bmp180, 0, MAVLINK_MSG_ID_ECO_BMP180_LEN);
    memcpy(eco_bmp180, _MAV_PAYLOAD(msg), len);
#endif
}
