#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <termios.h>   // for baudrate id's

#include <cstddef>
#include <cstdint>

// All constants taken from:
// https://data2.manualslib.com/pdf7/155/15471/1547026-kvh_industries/1775_imu.pdf?f6170753c78a4cdd76253f4f8ec6ee7f&take=binary

static constexpr uint8_t N_BYTES_HEADER          = 4;
static constexpr uint8_t N_BYTES_INERTIAL_SENSOR = 4;
static constexpr uint8_t N_BYTES_AUX_SENSOR      = 4;
static constexpr uint8_t N_BYTES_STATUS          = 1;
static constexpr uint8_t N_BYTES_COUNTER         = 1;
static constexpr uint8_t N_BYTES_CRC             = 4;

// scale factor to use when converting g's reported by IMU to m/s^2
static constexpr double GRAVITY = 9.80665;   // TODO(tushaar): VERIFYME

static constexpr double ACC_SCALE = GRAVITY;

//======================================================
// byte that signifies the start of a datagram
constexpr uint32_t Message_Format_A_HEADER = 0xFE'81'FF'55;
constexpr uint32_t Message_Format_B_HEADER = 0xFE'81'FF'56;
constexpr uint32_t Message_Format_C_HEADER = 0xFE'81'FF'57;

constexpr uint32_t HEADER = Message_Format_B_HEADER;

constexpr uint8_t DATAGRAM_SIZE = 40;

// Match the vehicle IMU configuration: 500 Hz at 460800 baud.
constexpr unsigned int BAUD_RATE = 460800;
constexpr uint32_t DATAGRAM_TRANSIT_TIME_us = DATAGRAM_SIZE * 10 * 1000000ULL / BAUD_RATE;
constexpr size_t SERIAL_BUFFER_SIZE = 4096;
constexpr size_t SERIAL_READ_SIZE = 2048;

#endif  // CONSTANTS_H
