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

#if defined(_WIN32)

#include "arch/win.h"

#include <stdexcept>
#include "serial.h"

using std::invalid_argument;

SerialArch::SerialArch(const std::string &port, uint32_t baudrate, uint32_t timeout)
: port_ (port), fd_(INVALID_HANDLE_VALUE), is_open_ (false), baudrate_ (baudrate), timeout(timeout)
{
    memset(&dcb_, 0, sizeof(dcb_));
    dcb_.DCBlength = sizeof(dcb_);
    fd_ = INVALID_HANDLE_VALUE;
    memset(&co_, 0, sizeof(co_));
    co_.ReadIntervalTimeout = 0;
    co_.ReadTotalTimeoutMultiplier = 0;
    co_.ReadTotalTimeoutConstant = 0;
    co_.WriteTotalTimeoutMultiplier = 0;
    co_.WriteTotalTimeoutConstant = 0;

    memset(&ro_, 0, sizeof(ro_));
    memset(&wo_, 0, sizeof(wo_));
    memset(&wait_o_, 0, sizeof(wait_o_));

    ro_.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    wo_.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    wait_o_.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    open();
}

SerialArch::~SerialArch()
{
    close();

    CloseHandle(ro_.hEvent);
    CloseHandle(wo_.hEvent);
    CloseHandle(wait_o_.hEvent);
}

void SerialArch::open()
{
    if(port_.empty())
    {
        throw invalid_argument("Empty port name.");
    }
    if(is_open_ == true)
    {
        throw Exception ("Serial port already open.");
    }

    fd_ = CreateFile(
            port_.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL
    );

    if(fd_ == INVALID_HANDLE_VALUE)
    {
        DWORD errno_ = GetLastError();
        throw Exception(strerror(errno_));
    }

    if (!SetupComm(fd_, SERIAL_RX_BUFFER_SIZE, SERIAL_TX_BUFFER_SIZE))
    {
        close();
        DWORD errno_ = GetLastError();
        throw Exception(strerror(errno_));
    }

    dcb_.BaudRate = baudrate_;
    dcb_.ByteSize = 8;
    dcb_.Parity   = NOPARITY;
    dcb_.StopBits = ONESTOPBIT;
    dcb_.fDtrControl = DTR_CONTROL_ENABLE;

    if (!SetCommState(fd_, &dcb_))
    {
        close();
        DWORD errno_ = GetLastError();
        throw Exception(strerror(errno_));
    }

    if (!SetCommTimeouts(fd_, &co_))
    {
        close();
        DWORD errno_ = GetLastError();
        throw Exception(strerror(errno_));
    }

    if (!SetCommMask(fd_, EV_RXCHAR | EV_ERR ))
    {
        close();
        DWORD errno_ = GetLastError();
        throw Exception(strerror(errno_));
    }

    if (!PurgeComm(fd_, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR ))
    {
        close();
        DWORD errno_ = GetLastError();
        throw Exception(strerror(errno_));
    }

    is_open_ = true;

    Sleep(30);
}

bool SerialArch::isOpen()
{
    return is_open_;
}

void SerialArch::close()
{
    SetCommMask(fd_, 0);
    ResetEvent(wait_o_.hEvent);

    CloseHandle(fd_);
    fd_ = INVALID_HANDLE_VALUE;

    is_open_ = false;
}

void SerialArch::flush()
{
    PurgeComm(fd_, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );

}

size_t SerialArch::write(const uint8_t *data, size_t length)
{
    DWORD error;
    DWORD tx_written = 0;

    if(is_open_ == false)
    {
        throw Exception ("Port not open.");
    }

    if(data == NULL || length == 0)
        return 0;

    if(ClearCommError(fd_, &error, NULL) && error > 0)
    {
        PurgeComm(fd_, PURGE_TXABORT | PURGE_TXCLEAR);
    }

    if(!WriteFile(fd_, data, static_cast<DWORD>(length), &tx_written, &wo_))
    {
        if(GetLastError() != ERROR_IO_PENDING)
        {
            DWORD errno_ = GetLastError();
            throw Exception(strerror(errno_));
        }
    }

    return (size_t) (tx_written);
}

size_t SerialArch::read(uint8_t *buf, size_t size)
{
    if(is_open_ == false)
    {
        throw Exception ("Port not open.");
    }
    DWORD rx_read = 0;

    if(!ReadFile(fd_, buf, size, &rx_read, &ro_))
    {
        DWORD errno_ = GetLastError();
        throw Exception(strerror(errno_));
    }
    return rx_read;
}

bool SerialArch::waitReadable()
{
    throw Exception ("Using Nonoverlapped I/O, waitReadable is not implemented on Windows.");
    return false;
}

#endif