
#ifndef DATAGRAM_PARSER_H
#define DATAGRAM_PARSER_H

#include <cstdint>

#include "EigenWrapper.h"

typedef uint8_t *buffer_iterator_t;

struct SensorData {
    Vector3 gyro;
    Vector3 accel;

    uint32_t imu_timestamp_us;  // Format B: microsecond timestamp from IMU, wraps ~71 min
    int16_t temperature_raw;    // Format B: raw temperature value

    uint8_t counter;
    uint32_t crc;
};

bool parseDatagram(buffer_iterator_t &it, SensorData &sensor_data);

#endif   // DATAGRAM_PARSER_H
