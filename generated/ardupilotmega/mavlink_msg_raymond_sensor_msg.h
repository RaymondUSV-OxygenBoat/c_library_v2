#pragma once
// MESSAGE RAYMOND_SENSOR_MSG PACKING

#define MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG 11045


typedef struct __mavlink_raymond_sensor_msg_t {
 uint32_t time_boot_ms; /*<  自系统启动起的时间（毫秒）*/
 float o2_saturation; /*<  溶解氧饱和度（%）*/
 float o2_concentration; /*<  溶解氧浓度（mg/L）*/
 float front_obstacle_distance; /*<  前避障距离(米)*/
 float back_obstacle_distance; /*<  后避障距离(米)*/
 float water_depth; /*<  水深(米)*/
 float water_temperature; /*<  水温(摄氏度)*/
} mavlink_raymond_sensor_msg_t;

#define MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN 28
#define MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN 28
#define MAVLINK_MSG_ID_11045_LEN 28
#define MAVLINK_MSG_ID_11045_MIN_LEN 28

#define MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC 109
#define MAVLINK_MSG_ID_11045_CRC 109



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RAYMOND_SENSOR_MSG { \
    11045, \
    "RAYMOND_SENSOR_MSG", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_raymond_sensor_msg_t, time_boot_ms) }, \
         { "o2_saturation", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_raymond_sensor_msg_t, o2_saturation) }, \
         { "o2_concentration", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_raymond_sensor_msg_t, o2_concentration) }, \
         { "front_obstacle_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_raymond_sensor_msg_t, front_obstacle_distance) }, \
         { "back_obstacle_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_raymond_sensor_msg_t, back_obstacle_distance) }, \
         { "water_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_raymond_sensor_msg_t, water_depth) }, \
         { "water_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_raymond_sensor_msg_t, water_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RAYMOND_SENSOR_MSG { \
    "RAYMOND_SENSOR_MSG", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_raymond_sensor_msg_t, time_boot_ms) }, \
         { "o2_saturation", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_raymond_sensor_msg_t, o2_saturation) }, \
         { "o2_concentration", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_raymond_sensor_msg_t, o2_concentration) }, \
         { "front_obstacle_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_raymond_sensor_msg_t, front_obstacle_distance) }, \
         { "back_obstacle_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_raymond_sensor_msg_t, back_obstacle_distance) }, \
         { "water_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_raymond_sensor_msg_t, water_depth) }, \
         { "water_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_raymond_sensor_msg_t, water_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a raymond_sensor_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms  自系统启动起的时间（毫秒）
 * @param o2_saturation  溶解氧饱和度（%）
 * @param o2_concentration  溶解氧浓度（mg/L）
 * @param front_obstacle_distance  前避障距离(米)
 * @param back_obstacle_distance  后避障距离(米)
 * @param water_depth  水深(米)
 * @param water_temperature  水温(摄氏度)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raymond_sensor_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float o2_saturation, float o2_concentration, float front_obstacle_distance, float back_obstacle_distance, float water_depth, float water_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, o2_saturation);
    _mav_put_float(buf, 8, o2_concentration);
    _mav_put_float(buf, 12, front_obstacle_distance);
    _mav_put_float(buf, 16, back_obstacle_distance);
    _mav_put_float(buf, 20, water_depth);
    _mav_put_float(buf, 24, water_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
#else
    mavlink_raymond_sensor_msg_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.o2_saturation = o2_saturation;
    packet.o2_concentration = o2_concentration;
    packet.front_obstacle_distance = front_obstacle_distance;
    packet.back_obstacle_distance = back_obstacle_distance;
    packet.water_depth = water_depth;
    packet.water_temperature = water_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
}

/**
 * @brief Pack a raymond_sensor_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms  自系统启动起的时间（毫秒）
 * @param o2_saturation  溶解氧饱和度（%）
 * @param o2_concentration  溶解氧浓度（mg/L）
 * @param front_obstacle_distance  前避障距离(米)
 * @param back_obstacle_distance  后避障距离(米)
 * @param water_depth  水深(米)
 * @param water_temperature  水温(摄氏度)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raymond_sensor_msg_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float o2_saturation, float o2_concentration, float front_obstacle_distance, float back_obstacle_distance, float water_depth, float water_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, o2_saturation);
    _mav_put_float(buf, 8, o2_concentration);
    _mav_put_float(buf, 12, front_obstacle_distance);
    _mav_put_float(buf, 16, back_obstacle_distance);
    _mav_put_float(buf, 20, water_depth);
    _mav_put_float(buf, 24, water_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
#else
    mavlink_raymond_sensor_msg_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.o2_saturation = o2_saturation;
    packet.o2_concentration = o2_concentration;
    packet.front_obstacle_distance = front_obstacle_distance;
    packet.back_obstacle_distance = back_obstacle_distance;
    packet.water_depth = water_depth;
    packet.water_temperature = water_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
#endif
}

/**
 * @brief Pack a raymond_sensor_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms  自系统启动起的时间（毫秒）
 * @param o2_saturation  溶解氧饱和度（%）
 * @param o2_concentration  溶解氧浓度（mg/L）
 * @param front_obstacle_distance  前避障距离(米)
 * @param back_obstacle_distance  后避障距离(米)
 * @param water_depth  水深(米)
 * @param water_temperature  水温(摄氏度)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raymond_sensor_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float o2_saturation,float o2_concentration,float front_obstacle_distance,float back_obstacle_distance,float water_depth,float water_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, o2_saturation);
    _mav_put_float(buf, 8, o2_concentration);
    _mav_put_float(buf, 12, front_obstacle_distance);
    _mav_put_float(buf, 16, back_obstacle_distance);
    _mav_put_float(buf, 20, water_depth);
    _mav_put_float(buf, 24, water_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
#else
    mavlink_raymond_sensor_msg_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.o2_saturation = o2_saturation;
    packet.o2_concentration = o2_concentration;
    packet.front_obstacle_distance = front_obstacle_distance;
    packet.back_obstacle_distance = back_obstacle_distance;
    packet.water_depth = water_depth;
    packet.water_temperature = water_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
}

/**
 * @brief Encode a raymond_sensor_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raymond_sensor_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raymond_sensor_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raymond_sensor_msg_t* raymond_sensor_msg)
{
    return mavlink_msg_raymond_sensor_msg_pack(system_id, component_id, msg, raymond_sensor_msg->time_boot_ms, raymond_sensor_msg->o2_saturation, raymond_sensor_msg->o2_concentration, raymond_sensor_msg->front_obstacle_distance, raymond_sensor_msg->back_obstacle_distance, raymond_sensor_msg->water_depth, raymond_sensor_msg->water_temperature);
}

/**
 * @brief Encode a raymond_sensor_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param raymond_sensor_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raymond_sensor_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_raymond_sensor_msg_t* raymond_sensor_msg)
{
    return mavlink_msg_raymond_sensor_msg_pack_chan(system_id, component_id, chan, msg, raymond_sensor_msg->time_boot_ms, raymond_sensor_msg->o2_saturation, raymond_sensor_msg->o2_concentration, raymond_sensor_msg->front_obstacle_distance, raymond_sensor_msg->back_obstacle_distance, raymond_sensor_msg->water_depth, raymond_sensor_msg->water_temperature);
}

/**
 * @brief Encode a raymond_sensor_msg struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param raymond_sensor_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raymond_sensor_msg_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_raymond_sensor_msg_t* raymond_sensor_msg)
{
    return mavlink_msg_raymond_sensor_msg_pack_status(system_id, component_id, _status, msg,  raymond_sensor_msg->time_boot_ms, raymond_sensor_msg->o2_saturation, raymond_sensor_msg->o2_concentration, raymond_sensor_msg->front_obstacle_distance, raymond_sensor_msg->back_obstacle_distance, raymond_sensor_msg->water_depth, raymond_sensor_msg->water_temperature);
}

/**
 * @brief Send a raymond_sensor_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms  自系统启动起的时间（毫秒）
 * @param o2_saturation  溶解氧饱和度（%）
 * @param o2_concentration  溶解氧浓度（mg/L）
 * @param front_obstacle_distance  前避障距离(米)
 * @param back_obstacle_distance  后避障距离(米)
 * @param water_depth  水深(米)
 * @param water_temperature  水温(摄氏度)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raymond_sensor_msg_send(mavlink_channel_t chan, uint32_t time_boot_ms, float o2_saturation, float o2_concentration, float front_obstacle_distance, float back_obstacle_distance, float water_depth, float water_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, o2_saturation);
    _mav_put_float(buf, 8, o2_concentration);
    _mav_put_float(buf, 12, front_obstacle_distance);
    _mav_put_float(buf, 16, back_obstacle_distance);
    _mav_put_float(buf, 20, water_depth);
    _mav_put_float(buf, 24, water_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG, buf, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
#else
    mavlink_raymond_sensor_msg_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.o2_saturation = o2_saturation;
    packet.o2_concentration = o2_concentration;
    packet.front_obstacle_distance = front_obstacle_distance;
    packet.back_obstacle_distance = back_obstacle_distance;
    packet.water_depth = water_depth;
    packet.water_temperature = water_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG, (const char *)&packet, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
#endif
}

/**
 * @brief Send a raymond_sensor_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_raymond_sensor_msg_send_struct(mavlink_channel_t chan, const mavlink_raymond_sensor_msg_t* raymond_sensor_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_raymond_sensor_msg_send(chan, raymond_sensor_msg->time_boot_ms, raymond_sensor_msg->o2_saturation, raymond_sensor_msg->o2_concentration, raymond_sensor_msg->front_obstacle_distance, raymond_sensor_msg->back_obstacle_distance, raymond_sensor_msg->water_depth, raymond_sensor_msg->water_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG, (const char *)raymond_sensor_msg, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
#endif
}

#if MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_raymond_sensor_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float o2_saturation, float o2_concentration, float front_obstacle_distance, float back_obstacle_distance, float water_depth, float water_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, o2_saturation);
    _mav_put_float(buf, 8, o2_concentration);
    _mav_put_float(buf, 12, front_obstacle_distance);
    _mav_put_float(buf, 16, back_obstacle_distance);
    _mav_put_float(buf, 20, water_depth);
    _mav_put_float(buf, 24, water_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG, buf, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
#else
    mavlink_raymond_sensor_msg_t *packet = (mavlink_raymond_sensor_msg_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->o2_saturation = o2_saturation;
    packet->o2_concentration = o2_concentration;
    packet->front_obstacle_distance = front_obstacle_distance;
    packet->back_obstacle_distance = back_obstacle_distance;
    packet->water_depth = water_depth;
    packet->water_temperature = water_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG, (const char *)packet, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_MIN_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_CRC);
#endif
}
#endif

#endif

// MESSAGE RAYMOND_SENSOR_MSG UNPACKING


/**
 * @brief Get field time_boot_ms from raymond_sensor_msg message
 *
 * @return  自系统启动起的时间（毫秒）
 */
static inline uint32_t mavlink_msg_raymond_sensor_msg_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field o2_saturation from raymond_sensor_msg message
 *
 * @return  溶解氧饱和度（%）
 */
static inline float mavlink_msg_raymond_sensor_msg_get_o2_saturation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field o2_concentration from raymond_sensor_msg message
 *
 * @return  溶解氧浓度（mg/L）
 */
static inline float mavlink_msg_raymond_sensor_msg_get_o2_concentration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field front_obstacle_distance from raymond_sensor_msg message
 *
 * @return  前避障距离(米)
 */
static inline float mavlink_msg_raymond_sensor_msg_get_front_obstacle_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field back_obstacle_distance from raymond_sensor_msg message
 *
 * @return  后避障距离(米)
 */
static inline float mavlink_msg_raymond_sensor_msg_get_back_obstacle_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field water_depth from raymond_sensor_msg message
 *
 * @return  水深(米)
 */
static inline float mavlink_msg_raymond_sensor_msg_get_water_depth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field water_temperature from raymond_sensor_msg message
 *
 * @return  水温(摄氏度)
 */
static inline float mavlink_msg_raymond_sensor_msg_get_water_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a raymond_sensor_msg message into a struct
 *
 * @param msg The message to decode
 * @param raymond_sensor_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_raymond_sensor_msg_decode(const mavlink_message_t* msg, mavlink_raymond_sensor_msg_t* raymond_sensor_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    raymond_sensor_msg->time_boot_ms = mavlink_msg_raymond_sensor_msg_get_time_boot_ms(msg);
    raymond_sensor_msg->o2_saturation = mavlink_msg_raymond_sensor_msg_get_o2_saturation(msg);
    raymond_sensor_msg->o2_concentration = mavlink_msg_raymond_sensor_msg_get_o2_concentration(msg);
    raymond_sensor_msg->front_obstacle_distance = mavlink_msg_raymond_sensor_msg_get_front_obstacle_distance(msg);
    raymond_sensor_msg->back_obstacle_distance = mavlink_msg_raymond_sensor_msg_get_back_obstacle_distance(msg);
    raymond_sensor_msg->water_depth = mavlink_msg_raymond_sensor_msg_get_water_depth(msg);
    raymond_sensor_msg->water_temperature = mavlink_msg_raymond_sensor_msg_get_water_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN? msg->len : MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN;
        memset(raymond_sensor_msg, 0, MAVLINK_MSG_ID_RAYMOND_SENSOR_MSG_LEN);
    memcpy(raymond_sensor_msg, _MAV_PAYLOAD(msg), len);
#endif
}
