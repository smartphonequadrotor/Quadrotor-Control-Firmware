/*****************************************************************************
Copyright (c) 2011 Abhin Chhabra, Jordan Woehr, Lisa Graham, Ryan Bray

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************/

#ifndef _QCFP_H_
#define _QCFP_H_

#include "qcb.h"

// Corresponds to the max size of the buffers in the event queue
#define QCFP_MAX_PACKET_SIZE 32

// Commands
#define QCFP_ASYNC_DATA           0x10
#define     QCFP_ASYNC_DATA_ACCEL 0x01
#define     QCFP_ASYNC_DATA_GYRO  0x02
#define     QCFP_ASYNC_DATA_BAT   0x04
#define     QCFP_ASYNC_DATA_EXP   0x08
#define     QCFP_ASYNC_DATA_MAG   0x10
#define QCFP_CALIBRATE_QUADROTOR  0x40
#define QCFP_FLIGHT_MODE          0x41
#define QCFP_RAW_MOTOR_CONTROL    0xF0
#define QCFP_NACK                 0xFF

void qcfp_init(void);
void qcfp_data_received(uint8_t buffer[], uint8_t buffer_size);
void qcfp_send_data(uint8_t buffer[], uint8_t buffer_size);
void qcfp_format_timestamp(uint8_t buffer[]);

#endif // _QCFP_H_
