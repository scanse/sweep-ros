/*The MIT License (MIT)
 *
 * Copyright (c) 2016, Scanse LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#if !defined(_WIN32)

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdexcept>
#include <string.h>

#include "arch/unix.h"

#include "serial.h"

using std::invalid_argument;

SerialArch::SerialArch(const std::string &port, uint32_t baudrate, uint32_t timeout)
        : port_(port), fd_(-1), is_open_(false), baudrate_(baudrate), timeout(timeout)
{
    open();
}

SerialArch::~SerialArch()
{
    close();
}

void SerialArch::open()
{
    if (port_.empty())
    {
        throw invalid_argument("Empty port name.");
    }
    if (isOpen() == true)
    {
        throw Exception("Serial port already open.");
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ == -1)
    {
        throw Exception(strerror(errno));
    }

    struct termios options; // The options for the file descriptor

    // Check if file descriptor is pointing to TTY device or not
    if (!isatty(fd_))
    {
        throw Exception(strerror(errno));
    }

    // Check the current options of device
    if (tcgetattr(fd_, &options) == -1)
    {
        throw Exception(strerror(errno));
    }

    // Input Flags
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK);
    // SW Flow Control OFF
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Output Flags
    options.c_oflag &= ~(OPOST);

    // Local Flags
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                         ISIG);

    //IEXTEN

    // Control Flags
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);

    // setup baud rate
    speed_t baud;
    baud = getbaud(baudrate_);

    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);

    // flush the port
    tcflush(fd_, TCIFLUSH);

    // set port attributes
    if (tcsetattr(fd_, TCSANOW, &options))
    {
        close();
        throw Exception(strerror(errno));
    }

    is_open_ = true;
}

bool SerialArch::isOpen()
{
    return is_open_;
}

void SerialArch::close()
{
    if (is_open_)
    {
        if (fd_ != -1)
        {
            int ret;
            ret = ::close(fd_);
            if (ret == 0)
            {
                fd_ = -1;
            }
            else
            {
                throw Exception(strerror(errno));
            }
        }
        is_open_ = false;
    }
}

void SerialArch::flush()
{
    tcflush(fd_, TCIFLUSH);
}

size_t SerialArch::write(const uint8_t *data, size_t length)
{
    if (is_open_ == false)
    {
        throw Exception("Port not open.");
    }

    if (data == NULL || length == 0) return 0;

    size_t tx_written = 0;

    while (tx_written < length)
    {
        int ans = ::write(fd_, data, length - tx_written);

        if (ans == -1) return tx_written;
        tx_written += ans;
    }

    return tx_written;

}

size_t SerialArch::read(uint8_t *buf, size_t size)
{
    size_t bytes_read = 0;

    if (is_open_ == false)
    {
        throw Exception("Port not open.");
    }

    ssize_t bytes_read_now = ::read(fd_, buf, size);
    if (bytes_read_now > 0)
    {
        bytes_read = bytes_read_now;
    }


    while (bytes_read < size)
    {
        if (waitReadable())
        {
            ssize_t bytes_read_now = ::read(fd_, buf + bytes_read, size - bytes_read);

            if (bytes_read_now < 1)
            {
                throw Exception("No bytes read when data when available");
            }

            bytes_read += bytes_read_now;
            if (bytes_read == size)
            {
                break;
            }
            // If bytes_read < size then we have more to read
            if (bytes_read < size)
            {
                continue;
            }
            if (bytes_read > size)
            {
                throw Exception("Too many bytes read");
            }
        }
        else
        {
            return TIMEOUT;
        }

    }

    return bytes_read;
}

bool SerialArch::waitReadable()
{
    // Setup a select call to block for serial data or a timeout
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    struct timeval timeout_val;

    /* Initialize the timeout structure */
    timeout_val.tv_sec = timeout / 1000;
    timeout_val.tv_usec = (timeout % 1000) * 1000;

    int r = ::select(fd_ + 1, &readfds, NULL, NULL, &timeout_val);

    if (r < 0)
    {
        // Select was interrupted
        if (errno == EINTR)
        {
            return false;
        }
        // Otherwise there was some error
        throw Exception(strerror(errno));
    }
    // Timeout occurred
    if (r == 0)
    {
        return false;
    }
    // This shouldn't happen, if r > 0 our fd has to be in the list!
    if (!FD_ISSET(fd_, &readfds))
    {
        throw Exception(strerror(errno));
    }
    // Data available to read.
    return true;
}

speed_t SerialArch::getbaud(uint32_t baudrate)
{
    speed_t baud;
    switch (baudrate)
    {
#ifdef B0
        case 0:
            baud = B0;
            break;
#endif
#ifdef B50
        case 50:
            baud = B50;
            break;
#endif
#ifdef B75
        case 75:
            baud = B75;
            break;
#endif
#ifdef B110
        case 110:
            baud = B110;
            break;
#endif
#ifdef B134
        case 134:
            baud = B134;
            break;
#endif
#ifdef B150
        case 150:
            baud = B150;
            break;
#endif
#ifdef B200
        case 200:
            baud = B200;
            break;
#endif
#ifdef B300
        case 300:
            baud = B300;
            break;
#endif
#ifdef B600
        case 600:
            baud = B600;
            break;
#endif
#ifdef B1200
        case 1200:
            baud = B1200;
            break;
#endif
#ifdef B1800
        case 1800:
            baud = B1800;
            break;
#endif
#ifdef B2400
        case 2400:
            baud = B2400;
            break;
#endif
#ifdef B4800
        case 4800:
            baud = B4800;
            break;
#endif
#ifdef B7200
        case 7200: baud = B7200; break;
#endif
#ifdef B9600
        case 9600:
            baud = B9600;
            break;
#endif
#ifdef B14400
        case 14400: baud = B14400; break;
#endif
#ifdef B19200
        case 19200:
            baud = B19200;
            break;
#endif
#ifdef B28800
        case 28800: baud = B28800; break;
#endif
#ifdef B57600
        case 57600:
            baud = B57600;
            break;
#endif
#ifdef B76800
        case 76800: baud = B76800; break;
#endif
#ifdef B38400
        case 38400:
            baud = B38400;
            break;
#endif
#ifdef B115200
        case 115200:
            baud = B115200;
            break;
#endif
#ifdef B128000
        case 128000: baud = B128000; break;
#endif
#ifdef B153600
        case 153600: baud = B153600; break;
#endif
#ifdef B230400
        case 230400:
            baud = B230400;
            break;
#endif
#ifdef B256000
        case 256000: baud = B256000; break;
#endif
#ifdef B460800
        case 460800:
            baud = B460800;
            break;
#endif
#ifdef B576000
        case 576000:
            baud = B576000;
            break;
#endif
#ifdef B921600
        case 921600:
            baud = B921600;
            break;
#endif
#ifdef B1000000
        case 1000000:
            baud = B1000000;
            break;
#endif
#ifdef B1152000
        case 1152000:
            baud = B1152000;
            break;
#endif
#ifdef B1500000
        case 1500000:
            baud = B1500000;
            break;
#endif
#ifdef B2000000
        case 2000000:
            baud = B2000000;
            break;
#endif
#ifdef B2500000
        case 2500000:
            baud = B2500000;
            break;
#endif
#ifdef B3000000
        case 3000000:
            baud = B3000000;
            break;
#endif
#ifdef B3500000
        case 3500000:
            baud = B3500000;
            break;
#endif
#ifdef B4000000
        case 4000000:
            baud = B4000000;
            break;
#endif
        default:
            throw invalid_argument("Baudrate cannot be set.");

    }

    return baud;
}

#endif