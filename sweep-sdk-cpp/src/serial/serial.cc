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

#include "serial.h"

#ifdef _WIN32
#include "arch/win.h"
#else

#include "arch/unix.h"

#endif


Serial::Serial(const std::string &port, uint32_t baudrate, uint32_t timeout)
        : parch_(new SerialArch(port, baudrate, timeout))
{

}

Serial::~Serial()
{
    delete parch_;
}

void Serial::open()
{
    parch_->open();
}

void Serial::close()
{
    parch_->close();
}

bool Serial::isOpen()
{
    return parch_->isOpen();
}

void Serial::flush()
{
    parch_->flush();
}

size_t Serial::write(const uint8_t *data, size_t length)
{
    return parch_->write(data, length);
}

size_t Serial::read(uint8_t *buf, size_t size)
{
    return parch_->read(buf, size);
}

bool Serial::waitReadable()
{
    return parch_->waitReadable();
}