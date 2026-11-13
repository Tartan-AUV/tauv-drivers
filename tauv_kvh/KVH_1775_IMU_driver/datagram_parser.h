
#ifndef DATAGRAM_PARSER_H
#define DATAGRAM_PARSER_H

#include <cstdint>
#include <cmath>

#include "EigenWrapper.h"

typedef uint8_t *buffer_iterator_t;

struct SensorData {
    Vector3 gyro;
    Vector3 accel;

    uint32_t imu_timestamp_us;  // Format B: microsecond timestamp from IMU, rollover is firmware dependent
    int16_t temperature_raw;    // Format B: raw temperature value

    uint8_t status;
    uint8_t counter;
    uint32_t crc;
};

inline bool gyroValid(const SensorData& sample) {
    return (sample.status & 0x07) == 0x07 && std::isfinite(sample.gyro.x) &&
           std::isfinite(sample.gyro.y) && std::isfinite(sample.gyro.z);
}

inline bool accelValid(const SensorData& sample, unsigned axis) {
    if (axis > 2) return false;
    const double values[] = {sample.accel.x, sample.accel.y, sample.accel.z};
    return (sample.status & (0x10u << axis)) && std::isfinite(values[axis]);
}

bool parseDatagram(buffer_iterator_t &it, SensorData &sensor_data);

#endif   // DATAGRAM_PARSER_H
