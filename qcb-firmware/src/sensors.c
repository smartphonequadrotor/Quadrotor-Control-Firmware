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

#include "sensors.h"
#include "twi.h"
#include "qcfp.h"
#include "pid/accel.h"
#include "pid/gyro.h"

static bool sensor_async_enabled = false;
static uint8_t sensors_calibration_state = SENSORS_UNCALIBRATED;

static void sensor_accel_init_complete(uint8_t buffer[], uint8_t length);
static void sensor_accel_sample(void);
static void sensor_accel_read_complete(uint8_t buffer[], uint8_t length);

static void sensor_gyro_init_complete(uint8_t buffer[], uint8_t length);
static void sensor_gyro_sample(void);
static void sensor_gyro_read_complete(uint8_t buffer[], uint8_t length);

static void sensor_mag_init_complete(uint8_t buffer[], uint8_t length);
static void sensor_mag_sample(void);
static void sensor_mag_read_complete(uint8_t buffer[], uint8_t length);

void sensors_init(void)
{
	// Initialize accelerometer
	twi_write_register(SENSOR_ACCEL_ADDR, ADXL345_POWER_CTL_ADDR, ADXL345_MEASURE, NULL);
	twi_write_register(SENSOR_ACCEL_ADDR, ADXL345_DATA_FORMAT_ADDR, ADXL345_FULL_RES | ADXL345_4G_RANGE, NULL);
	twi_write_register(SENSOR_ACCEL_ADDR, ADXL345_BW_RATE_ADDR, ADXL345_RATE_200, sensor_accel_init_complete);
}

void sensors_set_async(bool on)
{
	sensor_async_enabled = on;
}

void sensors_set_calibration(bool on)
{
	if(on)
	{
		sensors_calibration_state = SENSORS_CALIBRATED;
	}
	else
	{
		// Cancel calibration
	}
}

uint8_t sensors_get_calibration_state(void)
{
	return sensors_calibration_state;
}

static void sensor_accel_init_complete(uint8_t buffer[], uint8_t length)
{
	// Initialize gyroscope
	twi_write_register(SENSOR_GYRO_ADDR, ITG3200_RESET_ADDR, ITG3200_RESET, NULL);
	twi_write_register(SENSOR_GYRO_ADDR, ITG3200_LPF_ADDR, ITG3200_10HZ_LPF, NULL);
	twi_write_register(SENSOR_GYRO_ADDR, ITG3200_RESET_ADDR, ITG3200_X_GYRO_REF, sensor_gyro_init_complete);

	// Start sampling accelerometer
	eq_post_timer(sensor_accel_sample, SENSOR_ACCEL_SAMPLE_INTERVAL, eq_timer_periodic);
}

static void sensor_accel_sample(void)
{
	if(sensor_async_enabled || (sensors_calibration_state == SENSORS_CALIBRATING))
	{
		twi_read_register(SENSOR_ACCEL_ADDR, ADXL345_DATA_START, SENSOR_NUM_ACCEL_BYTES, sensor_accel_read_complete);
	}
}

static void sensor_accel_read_complete(uint8_t buffer[], uint8_t length)
{
	uint8_t async_data_cmd[12];
	async_data_cmd[0] = QCFP_ASYNC_DATA;
	async_data_cmd[1] = QCFP_ASYNC_DATA_ACCEL;
	qcfp_format_timestamp(&async_data_cmd[2]);
	async_data_cmd[6] = buffer[0];//X LSB
	async_data_cmd[7] = buffer[1];//X MSB
	async_data_cmd[8] = buffer[2];//Y LSB
	async_data_cmd[9] = buffer[3];//Y MSB
	async_data_cmd[10] = buffer[4];//Z LSB
	async_data_cmd[11] = buffer[5];//Z MSB

	//record accelerometer sample to sample buffer.
	record_accel_sample((buffer[1]<<8)|buffer[0], (buffer[3]<<8)|buffer[2], (buffer[5]<<8)|buffer[4] );

	qcfp_send_data(async_data_cmd, 12);

}

static void sensor_gyro_init_complete(uint8_t buffer[], uint8_t length)
{
	// Initialize magnetometer
	// Starts a single conversion. Default configuration is good otherwise
	twi_write_register(SENSOR_MAG_ADDR, HMC5843_MODE_ADDR, HMC5843_MODE_SINGLE_CONV, sensor_mag_init_complete);

	// Start sampling gyroscope
	eq_post_timer(sensor_gyro_sample, SENSOR_GYRO_SAMPLE_INTERVAL, eq_timer_periodic);
}

static void sensor_gyro_sample(void)
{
	if(sensor_async_enabled || (sensors_calibration_state == SENSORS_CALIBRATING))
	{
		twi_read_register(SENSOR_GYRO_ADDR, ITG3200_DATA_START, SENSOR_NUM_GYRO_BYTES, sensor_gyro_read_complete);
	}
}

static void sensor_gyro_read_complete(uint8_t buffer[], uint8_t length)
{
	uint8_t async_data_cmd[12];
	async_data_cmd[0] = QCFP_ASYNC_DATA;
	async_data_cmd[1] = QCFP_ASYNC_DATA_GYRO;
	qcfp_format_timestamp(&async_data_cmd[2]);
	async_data_cmd[6] = buffer[1];//X LSB
	async_data_cmd[7] = buffer[0];//X MSB
	async_data_cmd[8] = buffer[3];//Y LSB
	async_data_cmd[9] = buffer[2];//Y MSB
	async_data_cmd[10] = buffer[5];//Z LSB
	async_data_cmd[11] = buffer[4];//Z MSB

	//Aeroquad swaps the X and Y axis in their code for some reason with the 9doF board
	//we will too.
	record_gyro_sample((buffer[2]<<8)|buffer[3], (buffer[0]<<8)|buffer[1], (buffer[4]<<8)|buffer[5] );

	qcfp_send_data(async_data_cmd, 12);

}

static void sensor_mag_init_complete(uint8_t buffer[], uint8_t length)
{
	// Start sampling gyroscope
	eq_post_timer(sensor_mag_sample, SENSOR_MAG_SAMPLE_INTERVAL, eq_timer_periodic);
}

static void sensor_mag_sample(void)
{
	if(sensor_async_enabled || (sensors_calibration_state == SENSORS_CALIBRATING))
	{
		// Get the data from the last single conversion
		twi_read_register(SENSOR_MAG_ADDR, HMC5843_DATA_START, SENSOR_NUM_MAG_BYTES, sensor_mag_read_complete);
		// Start a new single conversion
		twi_write_register(SENSOR_MAG_ADDR, HMC5843_MODE_ADDR, HMC5843_MODE_SINGLE_CONV, NULL);
	}
}

static void sensor_mag_read_complete(uint8_t buffer[], uint8_t length)
{
	uint8_t async_data_cmd[12];
	async_data_cmd[0] = QCFP_ASYNC_DATA;
	async_data_cmd[1] = QCFP_ASYNC_DATA_MAG;
	qcfp_format_timestamp(&async_data_cmd[2]);
	async_data_cmd[6] = buffer[1];
	async_data_cmd[7] = buffer[0];
	async_data_cmd[8] = buffer[3];
	async_data_cmd[9] = buffer[2];
	async_data_cmd[10] = buffer[5];
	async_data_cmd[11] = buffer[4];
	qcfp_send_data(async_data_cmd, 12);
>>>>>>> 95da12f28527d93f4706ff05ac1e19807c117ed0
}
