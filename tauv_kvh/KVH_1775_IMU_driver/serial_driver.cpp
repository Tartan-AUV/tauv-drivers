#include "serial_driver.h"

#include <unistd.h>

#include <cstring>

SerialDriver::SerialDriver(const std::string& serial_port_name)
    : serial_port_name_(serial_port_name) {
}

void SerialDriver::resetPath(const std::string& new_serial_port_name) {
    serial_port_name_ = new_serial_port_name;
}

SerialDriver::~SerialDriver() {
    ::close(file_handle_);
}

bool SerialDriver::isPortOpened() const {
    return openedPort_;
}

const std::string& SerialDriver::getSerialPortName() const {
    return serial_port_name_;
}
