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
#include "gpio.h"
#include "system.h"
#include "pid/accel.h"
#include "pid/gyro.h"
#include "pid/compass.h"
#include "pid/kinematics.h"

#define SENSOR_STARTUP_DELAY    (20*SYSTEM_1_MS)

static uint8_t sensors_calibration_state = SENSORS_UNCALIBRATED;
static uint16_t number_of_accel_calibration_samples = 0;
static uint16_t number_of_gyro_calibration_samples = 0;
static bool accel_calibrated = false;
static bool gyro_calibrated = false;
static bool mag_sample_collected = false;

void sensors_check_calibration_complete(void);

static void sensors_init_delayed(void);

static void sensor_accel_who(uint8_t buffer[], uint8_t length);
static void sensor_accel_init_complete(uint8_t buffer[], uint8_t length);
static void sensor_accel_sample(void);
static void sensor_accel_read_complete(uint8_t buffer[], uint8_t length);

static void sensor_gyro_who(uint8_t buffer[], uint8_t length);
static void sensor_gyro_init_complete(uint8_t buffer[], uint8_t length);
static void sensor_gyro_sample(void);
static void sensor_gyro_read_complete(uint8_t buffer[], uint8_t length);

static void sensor_mag_who(uint8_t buffer[], uint8_t length);
static void sensor_mag_init_complete(uint8_t buffer[], uint8_t length);
static void sensor_mag_sample(void);
static void sensor_mag_read_complete(uint8_t buffer[], uint8_t length);

void sensors_init(void)
{
	eq_post_timer(sensors_init_delayed, SENSOR_STARTUP_DELAY, eq_timer_one_shot);
}

static void sensors_init_delayed(void)
{
	// Initialize accelerometer
	twi_read_register(SENSOR_ACCEL_ADDR, ADXL345_WHO_ADDR, 1, sensor_accel_who);
	twi_write_register(SENSOR_ACCEL_ADDR, ADXL345_POWER_CTL_ADDR, ADXL345_MEASURE, NULL);
	twi_write_register(SENSOR_ACCEL_ADDR, ADXL345_DATA_FORMAT_ADDR, ADXL345_FULL_RES | ADXL345_4G_RANGE, NULL);
	twi_write_register(SENSOR_ACCEL_ADDR, ADXL345_BW_RATE_ADDR, ADXL345_RATE_200, sensor_accel_init_complete);
	// Initialize gyroscope
	twi_read_register(SENSOR_GYRO_ADDR, ITG3200_WHO_ADDR, 1, sensor_gyro_who);
	twi_write_register(SENSOR_GYRO_ADDR, ITG3200_RESET_ADDR, ITG3200_RESET, NULL);
	twi_write_register(SENSOR_GYRO_ADDR, ITG3200_LPF_ADDR, ITG3200_10HZ_LPF, NULL);
	twi_write_register(SENSOR_GYRO_ADDR, ITG3200_RESET_ADDR, ITG3200_X_GYRO_REF, sensor_gyro_init_complete);
	// Initialize magnetometer
	twi_read_register(SENSOR_MAG_ADDR, HMC5843_WHO_ADDR, 3, sensor_mag_who);
	// Starts a single conversion. Default configuration is good otherwise
	twi_write_register(SENSOR_MAG_ADDR, HMC5843_MODE_ADDR, HMC5843_MODE_SINGLE_CONV, sensor_mag_init_complete);
}

void sensors_set_calibration(bool on)
{
	gpio_clear_leds(gpio_led_2);
	gyro_calibrated = false;
	accel_calibrated = false;
	mag_sample_collected = false;
	number_of_accel_calibration_samples = 0;
	number_of_gyro_calibration_samples = 0;

	if(on)
	{
		reset_accel_samples();
		reset_gyro_samples();
		sensors_calibration_state = SENSORS_CALIBRATING;
//		sensors_calibration_state = SENSORS_CALIBRATED;
//		gyro_calibrated = true;
//		accel_calibrated = true;
//		mag_sample_collected = true;
//		sensors_check_calibration_complete();
	}
	else
	{
		// Cancel calibration
		// Places system into an uncalibrated state. If this happens, another
		// calibration MUST be performed.
		sensors_calibration_state = SENSORS_UNCALIBRATED;
		kinematics_stop();
	}
}

uint8_t sensors_get_calibration_state(void)
{
	return sensors_calibration_state;
}

void sensors_check_calibration_complete(void)
{
	if(accel_calibrated && gyro_calibrated)// && mag_sample_collected)
	{
		// We have a magnetometer sample, initial roll/pitch should be 0 since
		// the quadrotor is calibrating
		// Read compass uses the sample and calculates a heading
		read_compass(0.0, 0.0);
		kinematics_init();
		sensors_calibration_state = SENSORS_CALIBRATED;
		gpio_set_leds(gpio_led_2);
		qcfp_send_calibration_state();
	}
}

static void sensor_accel_init_complete(uint8_t buffer[], uint8_t length)
{
	// Start sampling accelerometer
	eq_post_timer(sensor_accel_sample, SENSOR_ACCEL_SAMPLE_INTERVAL, eq_timer_periodic);
}

static void sensor_accel_sample(void)
{
	if((sensors_calibration_state == SENSORS_CALIBRATED) || (sensors_calibration_state == SENSORS_CALIBRATING))
	{
		twi_read_register(SENSOR_ACCEL_ADDR, ADXL345_DATA_START, SENSOR_NUM_ACCEL_BYTES, sensor_accel_read_complete);
	}
}

static void sensor_accel_read_complete(uint8_t buffer[], uint8_t length)
{
	//record accelerometer sample to sample buffer.
	record_accel_sample((buffer[3] << 8) | buffer[2], (buffer[1] << 8) | buffer[0], -((buffer[5] << 8) | buffer[4]));

	if((sensors_calibration_state == SENSORS_CALIBRATING) && (!accel_calibrated))
	{
		if(number_of_accel_calibration_samples == SAMPLECOUNT_A)
		{
			computeAccelBias();
			accel_calibrated = true;
			sensors_check_calibration_complete();
		}
		else
		{
			number_of_accel_calibration_samples++;
		}
	}
}

static void sensor_gyro_init_complete(uint8_t buffer[], uint8_t length)
{
	// Start sampling gyroscope
	eq_post_timer(sensor_gyro_sample, SENSOR_GYRO_SAMPLE_INTERVAL, eq_timer_periodic);
}

static void sensor_gyro_sample(void)
{
	if((sensors_calibration_state == SENSORS_CALIBRATED) || (sensors_calibration_state == SENSORS_CALIBRATING))
	{
		twi_read_register(SENSOR_GYRO_ADDR, ITG3200_DATA_START, SENSOR_NUM_GYRO_BYTES, sensor_gyro_read_complete);
	}
}

static void sensor_gyro_read_complete(uint8_t buffer[], uint8_t length)
{
	// 9 degree of freedom board has the sensors mounted such that the x and y of the gyro are swapped
	record_gyro_sample((buffer[2] << 8) | buffer[3], (buffer[0] << 8) | buffer[1], (buffer[4] << 8) | buffer[5]);

	if((sensors_calibration_state == SENSORS_CALIBRATING) && (!gyro_calibrated))
	{
		if(number_of_gyro_calibration_samples == SAMPLECOUNT_A)
		{
			computeGyroBias();
			gyro_calibrated = true;
			sensors_check_calibration_complete();
		}
		else
		{
			number_of_gyro_calibration_samples++;
		}
	}
}

static void sensor_mag_init_complete(uint8_t buffer[], uint8_t length)
{
	// Start sampling gyroscope
	eq_post_timer(sensor_mag_sample, SENSOR_MAG_SAMPLE_INTERVAL, eq_timer_periodic);
}

static void sensor_mag_sample(void)
{
	if((sensors_calibration_state == SENSORS_CALIBRATED) || (sensors_calibration_state == SENSORS_CALIBRATING))
	{
		// Get the data from the last single conversion
		twi_read_register(SENSOR_MAG_ADDR, HMC5843_DATA_START, SENSOR_NUM_MAG_BYTES, sensor_mag_read_complete);
		// Start a new single conversion
		twi_write_register(SENSOR_MAG_ADDR, HMC5843_MODE_ADDR, HMC5843_MODE_SINGLE_CONV, NULL);
	}
}

static void sensor_mag_read_complete(uint8_t buffer[], uint8_t length)
{
	mag_sample_collected = true;

	record_compass_sample((buffer[0] << 8) | buffer[1], -((buffer[2] << 8) | buffer[3]), -((buffer[4] << 8) | buffer[5]));

	if(sensors_calibration_state == SENSORS_CALIBRATING)
	{
		sensors_check_calibration_complete();
	}
	else if(sensors_calibration_state == SENSORS_CALIBRATED)
	{
#if defined MARG_KIN || defined DCM_KIN
		read_compass(get_kinematics_angle(XAXIS), get_kinematics_angle(YAXIS));
#endif
	}
}

static void sensor_accel_who(uint8_t buffer[], uint8_t length)
{
	if(buffer[0] == ADXL345_WHO_ID)
	{
		int i;
		i=0;
		i++;
	}
}

static void sensor_gyro_who(uint8_t buffer[], uint8_t length)
{
	if(buffer[0] == ITG3200_WHO_ID)
	{
		int i;
		i=0;
		i++;
	}
}

static void sensor_mag_who(uint8_t buffer[], uint8_t length)
{
	if((buffer[0] == HMC5843_WHO_ID1) && (buffer[1] == HMC5843_WHO_ID2) && (buffer[2] == HMC5843_WHO_ID3))
	{
		int i;
		i=0;
		i++;
	}
}
