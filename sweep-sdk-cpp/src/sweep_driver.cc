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

#include "sweep.h"
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <algorithm>


SweepDriver::SweepDriver() :
        _isConnected(false), _isScanning(false)
{
    _serial = NULL;
}

SweepDriver::~SweepDriver()
{
    disconnect();
    delete _serial;
}


status SweepDriver::connect(const char *port, uint32_t baud,
                            uint32_t timeout)
{
    if (isConnected())
        return S_OK;

    _serial = new Serial(port, baud,
                         timeout);

    if (_serial->isOpen())
    {
        _serial->flush();

        _isConnected = true;

        //Stop scanning if already in progress
        stopScan();

        return S_OK;
    }
    else
    {
        return S_FAIL;
    }
}

void SweepDriver::disconnect()
{
    if (!_isConnected)
        return;

    stopScan();

    _serial->close();
}

bool SweepDriver::isConnected()
{
    return _isConnected;
}

status SweepDriver::stopScan()
{
    if (!isConnected())
        return S_FAIL;

    char cmd[] =
            {'D', 'X'};

    if (_writeCommand(cmd))
    {
        return S_FAIL;
    }

    //Wait for Sweep to stop data output if it was outputting
    usleep(5000);

    //Clear any left over data
    _serial->flush();

    //Send stop cmd once more to confirm response header
    if (_writeCommand(cmd))
    {
        return S_FAIL;
    }

    sweep_response_header_t response_header;
    if (_readResponseHeader(&response_header))
    {
        return S_FAIL;
    }

    if (response_header.cmdByte1 != SWEEP_RESPONSE_TYPE_DATA &&
        response_header.cmdByte2 != SWEEP_RESPONSE_TYPE_DATA_STOP)
    {
        return S_FAIL;
    }

    //Assemble Checksum
    uint8_t tempSum = ((response_header.cmdStatusByte1
                        + response_header.cmdStatusByte2) & 0x3F) + 0x30;

    if (response_header.cmdSum != tempSum)
    {
        return S_FAIL;
    }

    _isScanning = false;

    return S_OK;
}

status SweepDriver::startScan()
{
    status p_stat;
    pthread_t t_attr;

    if (!isConnected())
        return S_FAIL;
    if (_isScanning)
        return S_OK;

    char cmd[] =
            {'D', 'S'};

    if (_writeCommand(cmd))
    {
        return S_FAIL;
    }

    sweep_response_header_t response_header;
    if (_readResponseHeader(&response_header))
    {
        return S_FAIL;
    }

    if (response_header.cmdByte1 != SWEEP_RESPONSE_TYPE_DATA &&
        response_header.cmdByte2 != SWEEP_RESPONSE_TYPE_DATA_START)
    {
        return S_FAIL;
    }

    //Assemble Checksum
    uint8_t tempSum = ((response_header.cmdStatusByte1
                        + response_header.cmdStatusByte2) & 0x3F) + 0x30;

    if (response_header.cmdSum != tempSum)
    {
        return S_FAIL;
    }

    _isScanning = true;

    p_stat = pthread_create(&t_attr, NULL, &SweepDriver::_pollThreadEntry, this);
    if (p_stat)
    {
        return S_FAIL;
    }

    return S_OK;
}

status SweepDriver::getCompleteScan(sweep_response_scan_packet_t *scan_buffer, size_t &count, uint32_t timeout)
{
    struct timeval tv;
    struct timespec ts;

    gettimeofday(&tv, NULL);
    ts.tv_sec = time(NULL) + timeout / 1000;
    ts.tv_nsec = tv.tv_usec * 1000 + 1000 * 1000 * (timeout % 1000);
    ts.tv_sec += ts.tv_nsec / (1000 * 1000 * 1000);
    ts.tv_nsec %= (1000 * 1000 * 1000);

    pthread_mutex_lock(&ml);

    if (pthread_cond_timedwait(&scan_ready, &ml, &ts) == 0)
    {
        if (_scan_packet_count == 0)
            return S_TIMEOUT;

        size_t size_to_copy = std::min(count, _scan_packet_count);

        memcpy(scan_buffer, _scan_packet_buf, size_to_copy * sizeof(sweep_response_scan_packet_t));

        count = size_to_copy;

        _scan_packet_count = 0;

        pthread_mutex_unlock(&ml);

        return S_OK;
    }
    else
    {
        return S_TIMEOUT;
    }
}

status SweepDriver::resetSweep()
{
    sweep_response_header_t resp;

    if (!isConnected())
        return S_FAIL;

    char cmd[] =
            {'R', 'R'};

    if (_writeCommand(cmd))
    {
        return S_FAIL;
    }

    if (_readResponse(&resp))
    {
        return S_FAIL;
    }

    if (resp.cmdByte1 != SWEEP_RESPONSE_TYPE_RESET && resp.cmdByte2 != SWEEP_RESPONSE_TYPE_RESET)
    {
        return S_FAIL;
    }

    //Assemble Checksum
    uint8_t tempSum = ((resp.cmdStatusByte1
                        + resp.cmdStatusByte2) & 0x3F) + 0x30;

    if (resp.cmdSum != tempSum)
    {
        return S_FAIL;
    }

    return S_OK;
}

status SweepDriver::changeMotorSpeed(uint8_t value)
{
    sweep_response_param_t resp;
    char cmd[4];
    char speed[2];

    if (!isConnected())
        return S_FAIL;

    _getSpeedValue(value, speed);

    strcpy(cmd, "MS");
    strcat(cmd, speed);

    if (_writeCommand(cmd, true))
    {
        return S_FAIL;
    }

    if (_readResponse(&resp))
    {
        return S_FAIL;
    }

    if (resp.cmdByte1 != SWEEP_RESPONSE_TYPE_MOTOR && resp.cmdByte2 != 'S')
    {
        return S_FAIL;
    }

    //Assemble Checksum
    uint8_t tempSum = ((resp.cmdStatusByte1
                        + resp.cmdStatusByte2) & 0x3F) + 0x30;

    if (resp.cmdSum != tempSum)
    {
        return S_FAIL;
    }

    return S_OK;
}

status SweepDriver::getDeviceInfo(sweep_response_info_device_t *info)
{
    if (!isConnected())
        return S_FAIL;

    char cmd[] =
            {'I', 'D'};

    if (_writeCommand(cmd))
    {
        return S_FAIL;
    }

    if (_readResponse(info))
    {
        return S_FAIL;
    }

    if (info->cmdByte1 != SWEEP_RESPONSE_TYPE_INFO && info->cmdByte2 != SWEEP_RESPONSE_TYPE_INFO_DEVICE)
    {
        return S_FAIL;
    }

    return S_OK;
}

status SweepDriver::getVersionInfo(sweep_response_info_version_t *info)
{
    if (!isConnected())
        return S_FAIL;

    char cmd[] =
            {'I', 'V'};

    if (_writeCommand(cmd))
    {
        return S_FAIL;
    }

    if (_readResponse(info))
    {
        return S_FAIL;
    }

    if (info->cmdByte1 != SWEEP_RESPONSE_TYPE_INFO && info->cmdByte2 != SWEEP_RESPONSE_TYPE_INFO_VERSION)
    {
        return S_FAIL;
    }

    return S_OK;
}

status SweepDriver::getMotorInfo(sweep_response_info_motor_t *info)
{
    if (!isConnected())
        return S_FAIL;

    char cmd[] =
            {'M', 'I'};

    if (_writeCommand(cmd))
    {
        return S_FAIL;
    }

    if (_readResponse(info))
    {
        return S_FAIL;
    }

    if (info->cmdByte1 != SWEEP_RESPONSE_TYPE_INFO && info->cmdByte2 != SWEEP_RESPONSE_TYPE_INFO_MOTOR)
    {
        return S_FAIL;
    }

    return S_OK;
}

void SweepDriver::printDeviceInfo()
{
    sweep_response_info_device_t data;
    getDeviceInfo(&data);

    printf("Bit rate: %c%c%c%c%c%c\n", data.bit_rate[0], data.bit_rate[1], data.bit_rate[2], data.bit_rate[3],
           data.bit_rate[4], data.bit_rate[5]);
    printf("Laser State: %c\n", data.laser_state);
    printf("Mode: %c\n", data.mode);
    printf("Diagnostic: %c\n", data.diagnostic);
    printf("Motor Speed: %c%cHz\n", data.motor_speed[0], data.motor_speed[1]);
    printf("Sample Rate: %c%c%c%c\n", data.sample_rate[0], data.sample_rate[1], data.sample_rate[2],
           data.sample_rate[3]);
}

void SweepDriver::printVersionInfo()
{
    sweep_response_info_version_t data;
    getVersionInfo(&data);

    printf("Model: %c%c%c%c%c\n", data.model[0], data.model[1], data.model[2], data.model[3], data.model[4]);
    printf("Protocol Version: %c.%c\n", data.protocol_major, data.protocol_min);
    printf("Firmware Version: %c.%c\n", data.firmware_major, data.firmware_minor);
    printf("Hardware Version: %c\n", data.hardware_version);
    printf("Serial Number: %c%c%c%c%c%c%c%c\n", data.serial_no[0], data.serial_no[1], data.serial_no[2],
           data.serial_no[3], data.serial_no[4], data.serial_no[5], data.serial_no[6], data.serial_no[7]);
}

void SweepDriver::printMotorInfo()
{
    sweep_response_info_motor_t data;
    getMotorInfo(&data);

    printf("Motor Speed: %c%cHz\n", data.motor_speed[0], data.motor_speed[1]);
}

void SweepDriver::printScanPacket(sweep_response_scan_packet_t packetbuffer)
{
    printf("sync: %1d | angle: %6.2f | range: %5d | sig strength: %3d\n", packetbuffer.sync_error,
           INT_TO_FLOAT(packetbuffer.angle), packetbuffer.distance, packetbuffer.signal_strength);
}

status SweepDriver::_writeCommand(char cmd[], bool param)
{
    if (!_isConnected)
        return S_FAIL;

    if (param)
    {
        uint8_t pkt_data[5];
        sweep_cmd_param_packet_t *pkt =
                reinterpret_cast<sweep_cmd_param_packet_t *>(pkt_data);

        pkt->cmdByte1 = cmd[0];
        pkt->cmdByte2 = cmd[1];
        pkt->cmdParamByte1 = cmd[2];
        pkt->cmdParamByte2 = cmd[3];
        pkt->cmdParamTerm = '\n';

        _serial->write(pkt_data, 5);

        return S_OK;
    }
    else
    {
        uint8_t pkt_data[3];
        sweep_cmd_packet_t *pkt =
                reinterpret_cast<sweep_cmd_packet_t *>(pkt_data);

        pkt->cmdByte1 = cmd[0];
        pkt->cmdByte2 = cmd[1];
        pkt->cmdParamTerm = '\n';

        _serial->write(pkt_data, 3);

        return S_OK;
    }

}

status SweepDriver::_readResponseHeader(sweep_response_header_t *header)
{
    int recvPos = 0;
    uint8_t recvBuffer[sizeof(sweep_response_header_t)];
    uint8_t *headerBuffer = reinterpret_cast<uint8_t *>(header);
    size_t recvSize = sizeof(sweep_response_header_t);

    if (_serial->read(recvBuffer, recvSize) == -1)
    {
        return TIMEOUT;
    }

    for (size_t pos = 0; pos < recvSize; ++pos)
    {
        uint8_t currentByte = recvBuffer[pos];

        headerBuffer[recvPos++] = currentByte;

        if (recvPos == sizeof(sweep_response_header_t))
        {
            return S_OK;
        }
    }

    return S_FAIL;
}

template<class ans>
status SweepDriver::_readResponse(ans *header)
{
    int recvPos = 0;
    uint8_t recvBuffer[sizeof(ans)];
    uint8_t *headerBuffer = reinterpret_cast<uint8_t *>(header);
    size_t recvSize = sizeof(ans);

    if (_serial->read(recvBuffer, recvSize) == -1)
    {
        return TIMEOUT;
    }

    for (size_t pos = 0; pos < recvSize; ++pos)
    {
        uint8_t currentByte = recvBuffer[pos];

        headerBuffer[recvPos++] = currentByte;

        if (recvPos == sizeof(ans))
        {
            return S_OK;
        }
    }

    return S_FAIL;
}

status SweepDriver::_readScanData(
        sweep_response_scan_packet_t *packetbuffer, size_t &count,
        uint32_t timeout)
{
    if (!_isConnected)
    {
        count = 0;
        return S_FAIL;
    }

    size_t recvNodeCount = 0;
    uint32_t startTs = gettime_ms();
    uint32_t waitTime;

    while ((waitTime = gettime_ms() - startTs) <= timeout && recvNodeCount < count)
    {
        sweep_response_scan_packet_t node;
        if (_readScanPacket(&node, timeout - waitTime))
        {
            return S_FAIL;
        }

        packetbuffer[recvNodeCount++] = node;

        if (recvNodeCount == count)
            return S_OK;
    }
    count = recvNodeCount;
    return S_FAIL;
}


status SweepDriver::_readScanPacket(sweep_response_scan_packet_t *node,
                                    uint32_t timeout)
{
    int recvPos = 0;
    uint32_t startTs = gettime_ms();
    uint8_t recvBuffer[sizeof(sweep_response_scan_packet_t)];
    uint8_t *packetbuffer = (uint8_t *) node;
    uint32_t checksum = 0;

    while ((gettime_ms() - startTs) <= timeout)
    {
        size_t recvSize = sizeof(sweep_response_scan_packet_t);

        if (_serial->read(recvBuffer, recvSize) == SWEEP_RESPONSE_SCAN_NODE_SIZE)
        {
            for (size_t pos = 0; pos < recvSize; ++pos)
            {
                uint8_t currentByte = recvBuffer[pos];

                packetbuffer[recvPos++] = currentByte;

                //Complete Scan Packet Check
                if (recvPos == sizeof(sweep_response_scan_packet_t))
                {
                    //Perform checksum
                    checksum = checksum % 255;

                    if (packetbuffer[recvPos - 1] == checksum)
                    {
                        return S_OK;
                    }
                    else
                    {
                        return S_FAIL;
                    }
                }
                else
                {
                    checksum += currentByte;
                }
            }
        }
        else
        {
            return S_FAIL;
        }

    }

    return S_TIMEOUT;
}

void *SweepDriver::_pollThreadEntry(void *object)
{
    // Call the member function on the object passed to the thread
    return static_cast<SweepDriver *>(object)->_pollScanData();
}

void *SweepDriver::_pollScanData()
{
    sweep_response_scan_packet_t local_buf[MIN_SCAN_PACKETS];
    size_t count = MIN_SCAN_PACKETS;
    sweep_response_scan_packet_t local_scan[MAX_SCAN_PACKETS];
    size_t scan_count = 0;
    status ans;
    memset(local_scan, 0, sizeof(local_scan));

    while (_isScanning)
    {
        /* wait for count amount of nodes */
        if ((ans = _readScanData(local_buf, count)))
        {
            if (ans != S_TIMEOUT)
            {
                _isScanning = false;
                return (void *) S_FAIL;
            }
        }

        for (size_t pos = 0; pos < count; ++pos)
        {

            if (local_buf[pos].sync_error == 1) //If the packet is the first of a scan ~0 deg
            {
                // only publish the data when it contains a full 360 degree scan
                if (local_scan[0].sync_error == 1 && scan_count <= 1024)
                {

                    pthread_mutex_lock(&ml);

                    memcpy(_scan_packet_buf, local_scan, scan_count * sizeof(sweep_response_scan_packet_t));

                    _scan_packet_count = scan_count;

                    pthread_cond_signal(&scan_ready);

                    pthread_mutex_unlock(&ml);
                }
                scan_count = 0; //Reset to 0 to begin full scan accumulation
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan))
                scan_count -= 1; // prevent overflow
        }
    }

    _isScanning = false;
    return S_OK;
}

void SweepDriver::_getSpeedValue(uint8_t value, char *speed)
{
    switch (value)
    {
        case 2:
            strcpy(speed, "02");
            break;
        case 3:
            strcpy(speed, "03");
            break;
        case 4:
            strcpy(speed, "04");
            break;
        case 5:
            strcpy(speed, "05");
            break;
        case 6:
            strcpy(speed, "06");
            break;
        case 7:
            strcpy(speed, "07");
            break;
        case 8:
            strcpy(speed, "08");
            break;
        case 9:
            strcpy(speed, "09");
            break;
        case 10:
            strcpy(speed, "10");
            break;

    }
}