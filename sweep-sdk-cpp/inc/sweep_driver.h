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

#pragma once

#include "sweep_protocol.h"
#include "serial/serial.h"

#define DEFAULT_TIMEOUT 3000  //3000 ms
#define MAX_SCAN_PACKETS 2048
#define MIN_SCAN_PACKETS 25

/*
 * Class that provides data access to a sweep scanner.
 */
class SweepDriver
{
public:
    /*
     * Creates a sweep driver object
     */
    SweepDriver();

    /* Destructor */
    virtual ~SweepDriver();

    /*
     * Opens the specified serial port if not already open
     */
    status connect(const char *port, uint32_t baud, uint32_t timeout =
    DEFAULT_TIMEOUT);

    /*
     * Stops scan if in progress and disconnects from serial port
     */
    void disconnect();

    /*
     * Gets the connection status
     */
    bool isConnected();

    /*
     * Stops scan
     */
    status stopScan();

    /*
     * Starts scan
     */
    status startScan();

    /*
     * Returns a new complete scan if available
     */
    status getCompleteScan(sweep_response_scan_packet_t *scan_buffer, size_t &count, uint32_t timeout = DEFAULT_TIMEOUT);

    /*
     * Resets Scanner
     */
    status resetSweep();

    /*
    * Changes scanner rotation speed
    */
    status changeMotorSpeed(uint8_t speed);

    /*
     * Retrieves Device Info packet from scanner
     */
    status getDeviceInfo(_sweep_response_info_device_t *info);

    /*
     * Retrieves Version Info packet from scanner
     */
    status getVersionInfo(sweep_response_info_version_t *info);

    /*
     * Retrieves Motor Info packet from scanner
     */
    status getMotorInfo(sweep_response_info_motor_t *info);

    /*
     * Retrieves Device Info packet and prints to stdout
     */
    void printDeviceInfo();

    /*
     * Retrieves Version Info packet and prints to stdout
     */
    void printVersionInfo();

    /*
     * Retrieves Motor Info packet and prints to stdout
     */
    void printMotorInfo();

    /*
    * Prints single scan packet data to stdout
    */
    void printScanPacket(sweep_response_scan_packet_t packetbuffer);

protected:
    /*
     * writes serial command to scanner
     */
    status _writeCommand(char cmd[], bool param = false);

    /*
     * Reads response header packet from scanner
     */
    status _readResponseHeader(sweep_response_header_t *header);

    /*
     * Reads response packet from scanner
     */
    template<class ans>
    status _readResponse(ans *header);

    /*
     * Reads a defined amount of scan packets
     */
    status _readScanData(sweep_response_scan_packet_t *packetbuffer, size_t &count, uint32_t timeout = DEFAULT_TIMEOUT);

    /*
     * Reads a single scan packet
     */
    status _readScanPacket(sweep_response_scan_packet_t *node,
                           uint32_t timeout = DEFAULT_TIMEOUT);

    /*
     * Thread entry for _pollScanData()
     */
    static void *_pollThreadEntry(void *object);

    /*
     * Continually polls for scan packets
     */
    void *_pollScanData();

    /*
     * Gets protocol speed value
     */
    void _getSpeedValue(uint8_t value, char *speed);

    bool _isConnected;              // State of Serial Connection

    bool _isScanning;               // State of scanner operation

    Serial *_serial;                // Serial port object

    sweep_response_scan_packet_t _scan_packet_buf[1024];
    size_t _scan_packet_count;
    pthread_t _thread;
    pthread_mutex_t ml;
    pthread_cond_t scan_ready;

};
