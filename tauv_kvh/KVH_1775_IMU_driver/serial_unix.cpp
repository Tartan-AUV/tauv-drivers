#include "serial_unix.h"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <unistd.h>

namespace {
speed_t baudId(unsigned int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef __APPLE__
        // Darwin represents baud rates directly, unlike Linux's termios constants.
        case 460800: return static_cast<speed_t>(460800);
        case 921600: return static_cast<speed_t>(921600);
#endif
        default: throw std::invalid_argument("Unsupported KVH host baud rate");
    }
}
}

SerialUnix::SerialUnix(const std::string& path, unsigned int baud)
    : SerialDriver(path), baud_rate_(baud) {
    (void)baudId(baud_rate_);  // Fail on unsupported configuration before starting threads.
}

void SerialUnix::disconnect() {
    if (file_handle_ >= 0) ::close(file_handle_);
    file_handle_ = -1;
    openedPort_ = false;
    buffer_.reset();
}

void SerialUnix::open() {
    disconnect();
    file_handle_ = ::open(serial_port_name_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (file_handle_ < 0) { ++statistics_.io_errors; return; }
    termios config{};
    if (tcgetattr(file_handle_, &config) != 0) {
        ++statistics_.io_errors; disconnect(); return;
    }
    cfmakeraw(&config);
    config.c_iflag &= ~(IGNCR | IXON | IXOFF | IXANY | IGNPAR);
    config.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
#ifdef CRTSCTS
    config.c_cflag &= ~CRTSCTS;
#endif
    config.c_cflag |= CS8 | CREAD | CLOCAL;
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 0;
    const auto speed = baudId(baud_rate_);
    if (cfsetispeed(&config, speed) != 0 || cfsetospeed(&config, speed) != 0 ||
        tcsetattr(file_handle_, TCSANOW, &config) != 0) {
        ++statistics_.io_errors; disconnect(); return;
    }
    // Do not flush unread input when applying attributes. Re-synchronize in the parser.
    openedPort_ = true;
    ++statistics_.connections;
}

void SerialUnix::reconnect() { open(); }

ssize_t SerialUnix::readBytes() {
    if (!openedPort_) {
        open();
        if (!openedPort_) return -1;
    }
    pollfd pfd{};
    pfd.fd = file_handle_;
    pfd.events = POLLIN;
    const int ready = ::poll(&pfd, 1, 100);  // Bound shutdown latency.
    if (ready == 0) return 0;
    if (ready < 0) {
        if (errno == EINTR) return 0;
        ++statistics_.io_errors; disconnect(); return -1;
    }
    // Drain any final readable bytes before handling a simultaneous hangup.
    if (!(pfd.revents & POLLIN)) {
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            ++statistics_.io_errors; disconnect(); return -1;
        }
        return 0;
    }
    uint8_t bytes[SERIAL_READ_SIZE];
    const size_t available = buffer_.capacity() - buffer_.size();
    if (available == 0) return 0;  // Never overwrite unparsed bytes.
    const ssize_t count = ::read(file_handle_, bytes, std::min(available, sizeof(bytes)));
    if (count < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        ++statistics_.io_errors; disconnect(); return -1;
    }
    if (count == 0) {
        if (pfd.revents & (POLLHUP | POLLERR)) {
            ++statistics_.io_errors; disconnect(); return -1;
        }
        return 0;
    }
    for (ssize_t i = 0; i < count; ++i) buffer_.add(bytes[i]);
    statistics_.bytes_received += static_cast<uint64_t>(count);
    return count;
}
