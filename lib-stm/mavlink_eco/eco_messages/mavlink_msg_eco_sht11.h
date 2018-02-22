#pragma once
// MESSAGE ECO_SHT11 PACKING

#define MAVLINK_MSG_ID_ECO_SHT11 193

MAVPACKED(
typedef struct __mavlink_eco_sht11_t {
 float temperature; /*< Temperature from SHT11 sensor [degC]*/
 float humidity; /*< Humidity from SHT11 sensor [percent]*/
 float dowpoint; /*< Dow point from SHT11 sensor [degC]*/
}) mavlink_eco_sht11_t;

#define MAVLINK_MSG_ID_ECO_SHT11_LEN 12
#define MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN 12
#define MAVLINK_MSG_ID_193_LEN 12
#define MAVLINK_MSG_ID_193_MIN_LEN 12

#define MAVLINK_MSG_ID_ECO_SHT11_CRC 201
#define MAVLINK_MSG_ID_193_CRC 201



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ECO_SHT11 { \
    193, \
    "ECO_SHT11", \
    3, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_eco_sht11_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_eco_sht11_t, humidity) }, \
         { "dowpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_eco_sht11_t, dowpoint) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ECO_SHT11 { \
    "ECO_SHT11", \
    3, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_eco_sht11_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_eco_sht11_t, humidity) }, \
         { "dowpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_eco_sht11_t, dowpoint) }, \
         } \
}
#endif

/**
 * @brief Pack a eco_sht11 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature Temperature from SHT11 sensor [degC]
 * @param humidity Humidity from SHT11 sensor [percent]
 * @param dowpoint Dow point from SHT11 sensor [degC]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_sht11_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float temperature, float humidity, float dowpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_SHT11_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, dowpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_SHT11_LEN);
#else
    mavlink_eco_sht11_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.dowpoint = dowpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_SHT11_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_SHT11;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN, MAVLINK_MSG_ID_ECO_SHT11_LEN, MAVLINK_MSG_ID_ECO_SHT11_CRC);
}

/**
 * @brief Pack a eco_sht11 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature Temperature from SHT11 sensor [degC]
 * @param humidity Humidity from SHT11 sensor [percent]
 * @param dowpoint Dow point from SHT11 sensor [degC]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_eco_sht11_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float temperature,float humidity,float dowpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_SHT11_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, dowpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ECO_SHT11_LEN);
#else
    mavlink_eco_sht11_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.dowpoint = dowpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ECO_SHT11_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ECO_SHT11;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN, MAVLINK_MSG_ID_ECO_SHT11_LEN, MAVLINK_MSG_ID_ECO_SHT11_CRC);
}

/**
 * @brief Encode a eco_sht11 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param eco_sht11 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_sht11_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_eco_sht11_t* eco_sht11)
{
    return mavlink_msg_eco_sht11_pack(system_id, component_id, msg, eco_sht11->temperature, eco_sht11->humidity, eco_sht11->dowpoint);
}

/**
 * @brief Encode a eco_sht11 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eco_sht11 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_eco_sht11_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_eco_sht11_t* eco_sht11)
{
    return mavlink_msg_eco_sht11_pack_chan(system_id, component_id, chan, msg, eco_sht11->temperature, eco_sht11->humidity, eco_sht11->dowpoint);
}

/**
 * @brief Send a eco_sht11 message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature Temperature from SHT11 sensor [degC]
 * @param humidity Humidity from SHT11 sensor [percent]
 * @param dowpoint Dow point from SHT11 sensor [degC]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_eco_sht11_send(mavlink_channel_t chan, float temperature, float humidity, float dowpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ECO_SHT11_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, dowpoint);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_SHT11, buf, MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN, MAVLINK_MSG_ID_ECO_SHT11_LEN, MAVLINK_MSG_ID_ECO_SHT11_CRC);
#else
    mavlink_eco_sht11_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.dowpoint = dowpoint;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_SHT11, (const char *)&packet, MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN, MAVLINK_MSG_ID_ECO_SHT11_LEN, MAVLINK_MSG_ID_ECO_SHT11_CRC);
#endif
}

/**
 * @brief Send a eco_sht11 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_eco_sht11_send_struct(mavlink_channel_t chan, const mavlink_eco_sht11_t* eco_sht11)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_eco_sht11_send(chan, eco_sht11->temperature, eco_sht11->humidity, eco_sht11->dowpoint);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_SHT11, (const char *)eco_sht11, MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN, MAVLINK_MSG_ID_ECO_SHT11_LEN, MAVLINK_MSG_ID_ECO_SHT11_CRC);
#endif
}

#if MAVLINK_MSG_ID_ECO_SHT11_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_eco_sht11_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float temperature, float humidity, float dowpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, humidity);
    _mav_put_float(buf, 8, dowpoint);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_SHT11, buf, MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN, MAVLINK_MSG_ID_ECO_SHT11_LEN, MAVLINK_MSG_ID_ECO_SHT11_CRC);
#else
    mavlink_eco_sht11_t *packet = (mavlink_eco_sht11_t *)msgbuf;
    packet->temperature = temperature;
    packet->humidity = humidity;
    packet->dowpoint = dowpoint;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ECO_SHT11, (const char *)packet, MAVLINK_MSG_ID_ECO_SHT11_MIN_LEN, MAVLINK_MSG_ID_ECO_SHT11_LEN, MAVLINK_MSG_ID_ECO_SHT11_CRC);
#endif
}
#endif

#endif

// MESSAGE ECO_SHT11 UNPACKING


/**
 * @brief Get field temperature from eco_sht11 message
 *
 * @return Temperature from SHT11 sensor [degC]
 */
static inline float mavlink_msg_eco_sht11_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field humidity from eco_sht11 message
 *
 * @return Humidity from SHT11 sensor [percent]
 */
static inline float mavlink_msg_eco_sht11_get_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field dowpoint from eco_sht11 message
 *
 * @return Dow point from SHT11 sensor [degC]
 */
static inline float mavlink_msg_eco_sht11_get_dowpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a eco_sht11 message into a struct
 *
 * @param msg The message to decode
 * @param eco_sht11 C-struct to decode the message contents into
 */
static inline void mavlink_msg_eco_sht11_decode(const mavlink_message_t* msg, mavlink_eco_sht11_t* eco_sht11)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    eco_sht11->temperature = mavlink_msg_eco_sht11_get_temperature(msg);
    eco_sht11->humidity = mavlink_msg_eco_sht11_get_humidity(msg);
    eco_sht11->dowpoint = mavlink_msg_eco_sht11_get_dowpoint(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ECO_SHT11_LEN? msg->len : MAVLINK_MSG_ID_ECO_SHT11_LEN;
        memset(eco_sht11, 0, MAVLINK_MSG_ID_ECO_SHT11_LEN);
    memcpy(eco_sht11, _MAV_PAYLOAD(msg), len);
#endif
}
