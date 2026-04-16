#include "serial_unix.h"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cstring>   // for strerror_l()
#include <iostream>

#include "constants.h"
#include "serial_driver.h"
#include "utils.h"

#define errnoToString(savedErrno) strerror_l(savedErrno, uselocale((locale_t)0))

using namespace std;

// Everything is learned from
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// "termios is the newer (now already a few decades old)
// Unix API for terminal I/O"

SerialUnix::SerialUnix(const std::string& serial_port_name)
    : SerialDriver(serial_port_name) {
}

void SerialUnix::open() {
    /*O_RDONLY: Opens the port for reading only
    O_NOCTTY: The port never becomes the controlling terminal of the process
    O_NDELAY: Use non-blocking I/O. On some systems this also means the RS232
    DCD signal line is ignored*/
    file_handle_   = ::open(serial_port_name_.c_str(),
                            O_RDONLY | O_NOCTTY | O_NDELAY | O_CLOEXEC);
    int savedErrno = errno;
    if (file_handle_ == -1) {
        cout << "IMU_DRIVER: Failed to close fd " << file_handle_ << ": "
             << errnoToString(savedErrno) << "\n";

        cout << "Failed to open IMU at " << serial_port_name_ << ": "
             << errnoToString(savedErrno) << "\n";
        openedPort_ = false;
        return;
    }

    // Check if the file descriptor is pointing to a TTY device or not.
    int retval = isatty(file_handle_);
    savedErrno = errno;
    if (retval != 1) {
        cout << "Serial port is not TTY device: " << errnoToString(savedErrno)
             << "\n";
        return;
    }

    // Get the current configuration of the serial interface
    retval     = tcgetattr(file_handle_, &config_);
    savedErrno = errno;
    if (retval != 0) {
        cout << "Couldn't retrieve current serial config: "
             << errnoToString(savedErrno) << "\n";
        return;
    }

    //
    // Input flags - Turn off input processing
    //
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config_.c_iflag &=
        ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    //
    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config_.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    config_.c_oflag = 0;

    //
    // No line processing
    //
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config_.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    //
    // Turn off character processing
    //
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config_.c_cflag &= ~(CSIZE | PARENB);
    config_.c_cflag |= CS8;
    // config_.c_cflag |= CSTOPB;   // 2 stop bits

    config_.c_cc[VMIN]  = 1;   // One input byte is enough to return from read()
    config_.c_cc[VTIME] = 0;   // Inter-character timer off

    // Communication speed
    retval     = cfsetispeed(&config_, BAUD_ID);
    savedErrno = errno;
    if (retval != 0) {
        cout << "Input baud rate couldn't be set: " << errnoToString(savedErrno)
             << "\n";
        return;
    }
    retval     = cfsetospeed(&config_, BAUD_ID);
    savedErrno = errno;
    if (retval != 0) {
        cout << "Output baud rate couldn't be set: "
             << errnoToString(savedErrno) << "\n";
        return;
    }

    // Finally, apply the configuration
    retval     = tcsetattr(file_handle_, TCSAFLUSH, &config_);
    savedErrno = errno;
    if (retval != 0) {
        cout << "Couldn't apply any serial settings: "
             << errnoToString(savedErrno) << "\n";
        return;
    }

    cout << "IMU connected successfully!\n";

    openedPort_ = true;

    /*TODO: do we need to set exclusive access to the serial device?
        Code to accomplish setting exclusive access:
       https://en.wikibooks.org/wiki/Serial_Programming/termios#Exclusive_Access
        */
}

void SerialUnix::reconnect() {
    if (0 <= file_handle_) {
        // only close the fd if it's nonnegative (i.e. open() returned a valid
        // fd and we arent just trying to reconnect because open failed
        // previously)
        int closeResult = ::close(file_handle_);
        int savedErrno  = errno;
        if (-1 == closeResult) {
            cout << "IMU_DRIVER: Failed to close fd " << file_handle_ << ": "
                 << errnoToString(savedErrno) << "\n";
        }
    }
    open();
}

static double last_hearbeat                = -1e99;   // [s]
static double hearbeat_period              = 1.0;     // [s]
static int bytes_read_since_last_heartbeat = 0;

ssize_t SerialUnix::readBytes() {
    double currTime = get_time();

    // poll to check if file descriptor is ready for reading
    struct pollfd pfd {};
    pfd.fd     = file_handle_;
    pfd.events = POLLIN;

    int ms_timeout = 1000;
    int pollResult = poll(&pfd, 1, ms_timeout);
    int savedErrno = errno;
    if (pollResult < 1) {   // error
        if (firstPollError or (currTime - timeOfLastPollErrMsgSent) > 5) {
            cout << "IMU Driver Serial Port poll() error: "
                 << errnoToString(savedErrno) << ". Attempting reconnect.\n";
            reconnect();
            timeOfLastPollErrMsgSent = currTime;
        }
        firstPollError = false;
        return pollResult;
    }

    // actually performing the read
    uint8_t tmp[DATAGRAM_SIZE];
    ssize_t readResult = read(file_handle_, tmp, DATAGRAM_SIZE);
    savedErrno         = errno;
    if (readResult < 1) {   // error
        if (firstReadError or (currTime - timeOfLastReadErrMsgSent) > 5) {
            cout << "IMU Driver Serial Port read() error: "
                 << errnoToString(savedErrno) << ". Attempting reconnect.\n";
            reconnect();
            timeOfLastReadErrMsgSent = currTime;
        }
        firstReadError = false;
        return readResult;
    }

    // copy from temporary read buffer into circular buffer
    for (int i = 0; i < DATAGRAM_SIZE and i < readResult; i++) {
        buffer_.add(tmp[i]);
    }

    bytes_read_since_last_heartbeat += min((int)DATAGRAM_SIZE, (int)readResult);
    double t       = get_time();
    double elapsed = t - last_hearbeat;
    if (elapsed > hearbeat_period) {
        // cout << "Reading packets at "
        //      << bytes_read_since_last_heartbeat / (DATAGRAM_SIZE * elapsed)
        //      << " Hz\n";
        bytes_read_since_last_heartbeat = 0;
        last_hearbeat                   = t;
    }

    return readResult;
}
