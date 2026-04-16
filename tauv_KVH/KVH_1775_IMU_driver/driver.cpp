
#include <unistd.h>

#include <cassert>
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
}

void Driver::forceState(bool checksum_is_ok, bool no_internal_error) {
    checksum_is_ok_    = checksum_is_ok;
    no_internal_error_ = no_internal_error;
}

// returns true if a new datagram was found in the byte stream. Whether the
// CRC check passes is another matter
bool Driver::tryReadingNewPacket() {
    int64_t bytesRead = serial_driver_.readBytes();
    if (bytesRead < 1) {
        return false;
    }

    bool foundDatagram = false;
    // try to find a full datagram in the circular buffer
    while (DATAGRAM_SIZE <= serial_driver_.buffer_.size()) {
        // remove bytes from buffer until it begins with the expected header
        while (not foundDatagram) {
            if (serial_driver_.buffer_.size() < DATAGRAM_SIZE) {
                return false;   // not enough bytes
            }

            // check to see if the first 4 bytes match the expected header
            int i = 0;
            uint32_t parsed_header{};
            for (uint8_t byte : serial_driver_.buffer_) {
                if (4 <= i) {
                    break;
                }
                parsed_header = (parsed_header << 8u) | (uint32_t)byte;
                i++;
            }
            foundDatagram = (parsed_header == HEADER);

            if (not foundDatagram) {
                serial_driver_.buffer_.remove();
            }
        }

        // copying to an array because later parsing code requires + and <
        // iterator operators, but idk how to define those operators
        // for a circular buffer. Array iterators are easier to work with.
        uint8_t tmp[DATAGRAM_SIZE];
        int i = 0;
        for (uint8_t byte : serial_driver_.buffer_) {
            if (DATAGRAM_SIZE <= i) {
                break;
            }
            tmp[i] = byte;
            i++;
        }

        // check status bytes for error and compute checksum
        auto* it = tmp + N_BYTES_HEADER;   // start parsing after the header
        no_internal_error_ = parseDatagram(it, sensor_data_);

        auto* begin       = tmp;
        uint32_t checksum = calculateChecksum(begin, it);
        checksum_is_ok_   = (checksum == sensor_data_.crc);
        if (checksum_is_ok_) {
            // clear the datagram that was just parsed
            for (int j = 0; j < DATAGRAM_SIZE; j++) {
                serial_driver_.buffer_.remove();
            }
            return true;
        }

        // checksum failed: the datagram was corrupted or the 1st byte
        // happened to match the start byte; remove it and reparse
        serial_driver_.buffer_.remove();
    }

    return foundDatagram;
}

constexpr uint8_t crc_buff_len = DATAGRAM_SIZE - N_BYTES_CRC;
uint32_t calculateChecksum(const buffer_iterator_t& begin,
                           const buffer_iterator_t& end) {
    uint8_t buff[crc_buff_len];
    std::copy(begin, end - N_BYTES_CRC, buff);
    crc_t crc = crc_init();
    for (unsigned char& byte : buff) {
        crc = crc_update(crc, &byte, 1);
    }
    crc = crc_finalize(crc);
    return crc;
}
