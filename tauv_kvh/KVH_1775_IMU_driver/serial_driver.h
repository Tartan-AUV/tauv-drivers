
#ifndef SERIAL_DRIVER_H
#define SERIAL_DRIVER_H

#include <string>

#include "circular_buffer.h"
#include "constants.h"

#define buffSize (2 * DATAGRAM_SIZE)

class SerialDriver {
   public:
    SerialDriver() = default;
    explicit SerialDriver(const std::string& serial_port_name);
    virtual void open()         = 0;
    virtual ssize_t readBytes() = 0;
    virtual void reconnect()    = 0;
    virtual ~SerialDriver();

    void resetPath(const std::string& new_serial_port_name);
    bool isPortOpened() const;
    const std::string& getSerialPortName() const;
    CircularBuffer<uint8_t, buffSize> buffer_;

   protected:
    std::string serial_port_name_;
    bool openedPort_ = false;
    int file_handle_{};

    bool firstReadError = true;
    bool firstPollError = true;

    double timeOfLastReadErrMsgSent{};
    double timeOfLastPollErrMsgSent{};
};

#endif   // SERIAL_DRIVER_H
