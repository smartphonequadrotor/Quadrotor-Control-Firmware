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

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "qcb.h"

#define SENSOR_GYRO_ADDR                 0x68
#define     ITG3200_LPF_ADDR             0x16
#define         ITG3200_10HZ_LPF         0x1D
#define     ITG3200_RESET_ADDR           0x3E
#define         ITG3200_RESET            0x80
#define         ITG3200_X_GYRO_REF       0x01
#define     ITG3200_DATAX0_ADDR          0x1D
#define     ITG3200_DATA_START           ITG3200_DATAX0_ADDR
#define SENSOR_GYRO_SAMPLE_INTERVAL      5
#define SENSOR_NUM_GYRO_BYTES            6

#define SENSOR_ACCEL_ADDR                0x53
#define     ADXL345_BW_RATE_ADDR         0x2C
#define         ADXL345_RATE_400         0x0C
#define         ADXL345_RATE_200         0x0B
#define         ADXL345_RATE_100         0x0A
#define         ADXL345_RATE_50          0x09
#define     ADXL345_POWER_CTL_ADDR       0x2D
#define         ADXL345_MEASURE          (1 << 3)
#define     ADXL345_DATA_FORMAT_ADDR     0x31
#define         ADXL345_FULL_RES         0x08
#define         ADXL345_4G_RANGE         0x01
#define     ADXL345_DATAX0_ADDR          0x32
#define     ADXL345_DATA_START           ADXL345_DATAX0_ADDR
#define SENSOR_ACCEL_SAMPLE_INTERVAL     5
#define SENSOR_NUM_ACCEL_BYTES           6

#define SENSOR_MAG_ADDR                  0x1E
#define     HMC5843_MODE_ADDR            0x02
#define         HMC5843_MODE_SINGLE_CONV 0x01
#define     HMC5843_DATAX0_ADDR          0x03
#define     HMC5843_DATA_START           HMC5843_DATAX0_ADDR
#define SENSOR_MAG_SAMPLE_INTERVAL       101
#define SENSOR_NUM_MAG_BYTES             6

#define SENSORS_UNCALIBRATED 0x00
#define SENSORS_CALIBRATED   0x01
#define SENSORS_CALIBRATING  0x02

void sensors_init(void);
void sensors_set_calibration(bool on);
uint8_t sensors_get_calibration_state(void);

#endif // _SENSORS_H_
