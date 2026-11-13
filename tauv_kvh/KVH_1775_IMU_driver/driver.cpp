
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdint>

#include "constants.h"
extern "C" {
#include "crc.h"
}

#include "datagram_parser.h"
#include "driver.h"

using namespace std;

Driver::Driver(SerialDriver& serial_driver)
    : serial_driver_(serial_driver),
      checksum_is_ok_(false),
      no_internal_error_(true),
      sensor_data_() {
    serial_driver_.open();
}

Vector3 Driver::getAccData() const {
    return this->sensor_data_.accel;
}

Vector3 Driver::getGyroData() const {
    return this->sensor_data_.gyro;
}

std::string Driver::getDevicePath() const {
    return serial_driver_.getSerialPortName();
}

void Driver::setDevicePath(const char* newPath) {
    serial_driver_.resetPath(newPath);
}

bool Driver::isChecksumGood() const {
    return checksum_is_ok_;
}

bool Driver::isSensorStatusGood() const {
    return no_internal_error_;
}

uint32_t Driver::getTimestampUs() const {
    return sensor_data_.imu_timestamp_us;
}

void Driver::reset() {
    checksum_is_ok_    = false;
    no_internal_error_ = true;
    serial_driver_.buffer_.reset();
    have_sequence_ = false;
}

void Driver::forceState(bool checksum_is_ok, bool no_internal_error) {
    checksum_is_ok_    = checksum_is_ok;
    no_internal_error_ = no_internal_error;
}

uint8_t Driver::getInternalMeasurmentCounter() const { return sensor_data_.counter; }

bool Driver::tryReadingNewPacket() {
    if (connection_ != serial_driver_.statistics().connections) {
        connection_ = serial_driver_.statistics().connections;
        have_sequence_ = false;
    }
    // Consume a burst already in memory before polling the serial device again.
    if (parseBufferedPacket()) return true;
    if (serial_driver_.readBytes() < 1) return false;
    if (connection_ != serial_driver_.statistics().connections) {
        connection_ = serial_driver_.statistics().connections;
        have_sequence_ = false;
    }
    return parseBufferedPacket();
}

bool Driver::parseBufferedPacket() {
    auto& buffer = serial_driver_.buffer_;
    while (buffer.size() >= DATAGRAM_SIZE) {
        uint32_t header = 0;
        auto it_buffer = buffer.begin();
        for (int i = 0; i < N_BYTES_HEADER; ++i, ++it_buffer) header = (header << 8u) | *it_buffer;
        if (header != HEADER) {
            buffer.remove(); ++statistics_.discarded_bytes; continue;
        }
        std::array<uint8_t, DATAGRAM_SIZE> bytes{};
        std::copy_n(buffer.begin(), DATAGRAM_SIZE, bytes.begin());
        auto* it = bytes.data() + N_BYTES_HEADER;
        SensorData candidate{};
        const bool status_ok = parseDatagram(it, candidate);
        if (calculateChecksum(bytes.data(), it) != candidate.crc) {
            ++statistics_.crc_failures;
            buffer.remove(); ++statistics_.discarded_bytes;
            continue;  // Search for the full header again after every failed candidate.
        }
        for (int i = 0; i < DATAGRAM_SIZE; ++i) buffer.remove();
        sensor_data_ = candidate;
        checksum_is_ok_ = true;
        no_internal_error_ = status_ok;
        ++statistics_.valid_packets;
        if (!status_ok) ++statistics_.status_failures;
        if (have_sequence_) {
            const unsigned delta = (candidate.counter - last_sequence_) & 0x7f;
            if (delta == 0) ++statistics_.duplicate_packets;
            else statistics_.sequence_gaps += delta - 1;
        }
        have_sequence_ = true;
        last_sequence_ = candidate.counter;
        return true;
    }
    checksum_is_ok_ = false;
    return false;
}

uint32_t calculateChecksum(const buffer_iterator_t& begin,
                           const buffer_iterator_t& end) {
    return static_cast<uint32_t>(crc_finalize(crc_update(
        crc_init(), begin, static_cast<size_t>(end - begin - N_BYTES_CRC))));
}
