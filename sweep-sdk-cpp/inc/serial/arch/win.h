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

#ifndef WIN_H
#define WIN_H

#include <windows.h>
#include <v8stdint.h>
#include <iostream>

class SerialArch {
    public:
    enum{
        SERIAL_RX_BUFFER_SIZE = 512,
        SERIAL_TX_BUFFER_SIZE = 128
    };
        SerialArch(const std::string &port, uint32_t baudrate, uint32_t timeout);

        virtual ~SerialArch();

        void open();
        void close();
        bool isOpen();
        void flush();
        size_t write(const uint8_t *data, size_t length);
        size_t read(uint8_t *buf, size_t size);
        bool waitReadable();


    private:
        std::string port_;
        uint32_t baudrate_;
        HANDLE fd_;
        bool is_open_;
        uint32_t timeout;
        DCB dcb_;
        COMMTIMEOUTS co_;
        OVERLAPPED wait_o_;
        OVERLAPPED ro_, wo_;

};

#endif // WIN_H

#endif // defined(_WIN32)
