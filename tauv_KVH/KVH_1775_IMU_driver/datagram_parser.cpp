
#include "datagram_parser.h"

#include <cstdint>
#include <cstring>

#include "constants.h"

// Meta data is stored as "unsigned word", we simply combine the bytes into
// the right sized uint by left shifting them. Note the biggest is the CRC
// which is 32 bits.
static uint32_t parseUnsigned(buffer_iterator_t& it, uint8_t size) {
    uint32_t tmp{0};
    auto* end = it + size;
    while (it < end) {
        tmp = (tmp << 8u) | *(it++);
    }
    return tmp;
}

// SPFP stands for "Single Precision Floating Point".
// Use memcpy to avoid strict-aliasing UB when reinterpreting the bit pattern.
static float parseSPFP(buffer_iterator_t& it) {
    uint32_t bytes = (static_cast<uint32_t>(it[0]) << 24u) |
                     (static_cast<uint32_t>(it[1]) << 16u) |
                     (static_cast<uint32_t>(it[2]) << 8u) |
                     static_cast<uint32_t>(it[3]);
    it += 4;
    float result;
    std::memcpy(&result, &bytes, sizeof(float));
    return result;
}

// Parses Format B (header 0xFE81FF56, 40 bytes total).
// Iterator starts just after the 4-byte header.
// Returns true if the status byte indicates all sensors are valid (0x77).
bool parseDatagram(buffer_iterator_t& it, SensorData& sensor_data) {
    sensor_data.gyro.x = parseSPFP(it);
    sensor_data.gyro.y = parseSPFP(it);
    sensor_data.gyro.z = parseSPFP(it);

    sensor_data.accel.x = ACC_SCALE * parseSPFP(it);
    sensor_data.accel.y = ACC_SCALE * parseSPFP(it);
    sensor_data.accel.z = ACC_SCALE * parseSPFP(it);

    // Format B: 32-bit microsecond timestamp (wraps every ~71 minutes)
    sensor_data.imu_timestamp_us = parseUnsigned(it, 4);

    uint8_t status       = parseUnsigned(it, N_BYTES_STATUS);
    sensor_data.counter  = parseUnsigned(it, N_BYTES_COUNTER);

    // Format B: 16-bit signed temperature
    sensor_data.temperature_raw = static_cast<int16_t>(parseUnsigned(it, 2));

    sensor_data.crc = parseUnsigned(it, N_BYTES_CRC);

    const uint8_t nominal_status = 0b0111'0111;
    return status == nominal_status;
}
