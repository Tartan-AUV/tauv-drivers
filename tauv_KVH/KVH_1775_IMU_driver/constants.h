#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <termios.h>   // for baudrate id's

#include <cmath>
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

// Baudrate
constexpr auto BAUD_ID   = B921600;
constexpr auto BAUD_RATE = 921600;   // [bits/s]

// assume 10 bits/serial_byte (1 start bit + 8 bits/data_byte + 1 stop bit)
constexpr uint32_t DATAGRAM_TRANSIT_TIME_us =
    (uint32_t)((DATAGRAM_SIZE * 10) / (BAUD_RATE / 1e6));   // [us]

#endif   // CONSTANTS_H
