
#ifndef SERIAL_DRIVER_H
#define SERIAL_DRIVER_H

#include <string>
#include <sys/types.h>

#include "circular_buffer.h"
#include "constants.h"

struct SerialStatistics {
    uint64_t bytes_received = 0;
    uint64_t connections = 0;
    uint64_t io_errors = 0;
};

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
    CircularBuffer<uint8_t, SERIAL_BUFFER_SIZE> buffer_;
    const SerialStatistics& statistics() const { return statistics_; }

   protected:
    std::string serial_port_name_;
    bool openedPort_ = false;
    int file_handle_ = -1;
    SerialStatistics statistics_;

};

#endif   // SERIAL_DRIVER_H
