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

#include "qcfp.h"
#include "system.h"
#include "eq.h"
#include "pwm.h"
#include "us1.h"
#include "gpio.h"
#include "sensors.h"
#include "pid/kinematics.h"
#include "pid/pid.h"
#include "pid/gyro.h"
#include "pid/compass.h"
#include "pid/flight_controller.h"

// Max encoded packet size includes an extra byte at the start and end and
// up to 1 additional byte of overhead
// An encoded packet will never exceed 255 bytes
#define QCFP_MAX_ENCODED_PACKET_SIZE (QCFP_MAX_PACKET_SIZE+3)

#define COBS_TERM_BYTE (0)

#define ESC_STARTUP_TIME         5*SYSTEM_1_S

static bool flight_mode = false;

#define QCFP_CONTROL_MODE_NORMAL 0
#define QCFP_CONTROL_MODE_MANUAL 1
#define QCFP_CONTROL_MODE_PID    2

static uint8_t control_mode = QCFP_CONTROL_MODE_NORMAL;

// All handlers execute at the main loop level. Instead of using the
// stack, they can all share this buffer for assembling their responses.
static uint8_t response_buffer[QCFP_MAX_PACKET_SIZE];
static uint8_t response_length;

static void qcfp_handle_packet(uint8_t packet[], uint8_t length);

static bool qcfp_set_throttle(uint8_t payload[], uint8_t length);
static bool qcfp_set_desired_angles(uint8_t payload[], uint8_t length);
static bool qcfp_set_increment_height(uint8_t payload[], uint8_t length);
static bool qcfp_calibrate_quadrotor_handler(uint8_t payload[], uint8_t length);
static bool qcfp_flight_mode_handler(uint8_t payload[], uint8_t length);
static bool qcfp_raw_motor_control_handler(uint8_t payload[], uint8_t length);
static bool qcfp_control_method_override_handler(uint8_t payload[], uint8_t length);

void qcfp_init(void)
{

}

void qcfp_data_received(uint8_t buffer[], uint8_t buffer_size)
{
	typedef enum cobs_state
	{
		COBS_DECODE,
		COBS_COPY,
		COBS_SYNC,
	} cobs_state;

	static cobs_state decode_state = COBS_SYNC;
	static uint8_t incoming_packet[QCFP_MAX_PACKET_SIZE];
	static uint8_t packet_size = 0; // Counts packet size
	static uint8_t byte_count = 0; // Counts number of encoded bytes

	int i; // Must be signed because of i-- operation

	// Decode data from buffer
	for(i = 0; i < buffer_size; i++)
	{
		if(packet_size > QCFP_MAX_PACKET_SIZE)
		{
			decode_state = COBS_SYNC;
		}

		switch(decode_state)
		{
		case COBS_DECODE:
			if(buffer[i] == COBS_TERM_BYTE)
			{
				if((packet_size > 0) && (byte_count == 0))
				{
					if(incoming_packet[packet_size] == COBS_TERM_BYTE)
					{
						packet_size--;
					}
					eq_post(qcfp_handle_packet, incoming_packet, packet_size);
				}
				packet_size = 0;
				byte_count = 0;
			}
			else
			{
				byte_count = buffer[i];
				decode_state = COBS_COPY;
			}
			break;
		case COBS_COPY:
			if(byte_count == 1)
			{
				incoming_packet[packet_size++] = 0;
				i--;
				byte_count--;
				decode_state = COBS_DECODE;
			}
			else
			{
				if(buffer[i] == COBS_TERM_BYTE)
				{
					// Got a zero when expecting data, re-sync
					byte_count = 0;
					packet_size = 0;
					decode_state = COBS_DECODE;
				}
				else
				{
					if(byte_count > 1)
					{
						incoming_packet[packet_size++] = buffer[i];
						byte_count--;
					}
				}
			}
			break;
		case COBS_SYNC:
		default:
			packet_size = 0;
			byte_count = 0;
			if(buffer[i] == COBS_TERM_BYTE)
			{
				decode_state = COBS_DECODE;
			}
			break;
		}
	}
}

void qcfp_send_data(uint8_t buffer[], uint8_t buffer_size)
{
	uint8_t encoded_data[QCFP_MAX_ENCODED_PACKET_SIZE];
	uint8_t encoded_data_index = 1, chunk_index = 1, byte_count = 1;
	uint8_t i;

	if(buffer_size > QCFP_MAX_PACKET_SIZE)
	{
		return;
	}

	// First byte is always 0
	encoded_data[0] = COBS_TERM_BYTE;

	for(i = 0; i < buffer_size; i++, byte_count++)
	{
		if(buffer[i] == COBS_TERM_BYTE)
		{
			encoded_data[chunk_index] = byte_count;
			chunk_index = ++encoded_data_index;
			byte_count = 0;
		}
		else
		{
			encoded_data[++encoded_data_index] = buffer[i];
		}
	}

	if(byte_count > 1)
	{
		encoded_data[chunk_index] = byte_count;
		encoded_data_index++;
	}

	if(buffer[buffer_size-1] == COBS_TERM_BYTE)
	{
		encoded_data[encoded_data_index++] = 1;
	}

	encoded_data[encoded_data_index++] = COBS_TERM_BYTE;
	us1_send_buffer(encoded_data, encoded_data_index);
}

void qcfp_format_timestamp(uint8_t buffer[])
{
	uint32_t now = system_uptime();
	for(int i = 0; i < 4; i++)
	{
		buffer[i] = (now >> i*8) & 0x000000FF;
	}
}

void qcfp_format_float_as_bytes(uint8_t buffer[], float f)
{
	typedef union f_b
	{
		float f;
		uint8_t b[0];
	} f_b;

	f_b float_to_bytes;
	float_to_bytes.f = f;
	for(int i = 0; i < sizeof(f_b); i++)
	{
		buffer[i] = float_to_bytes.b[i];
	}
}

float qcfp_format_bytes_as_float(uint8_t buffer[])
{
	typedef union f_b
	{
		float f;
		uint8_t b[0];
	} f_b;

	f_b float_to_bytes;
	for(int i = 0; i < sizeof(f_b); i++)
	{
		float_to_bytes.b[i] = buffer[i];
	}
	return float_to_bytes.f;
}

bool qcfp_flight_enabled(void)
{
	return flight_mode;
}

bool qcfp_pid_enabled(void)
{
	return (control_mode == QCFP_CONTROL_MODE_PID);
}

void qcfp_send_calibration_state(void)
{
	uint8_t buffer[2];
	buffer[0] = QCFP_CALIBRATE_QUADROTOR;
	buffer[1] = sensors_get_calibration_state();
	qcfp_send_data(buffer, sizeof(buffer));
}

void qcfp_send_kinematics_angles(void)
{
	uint8_t buffer[18];
	if(sensors_get_calibration_state() == SENSORS_CALIBRATED)
	{
		buffer[0] = QCFP_ASYNC_DATA;
		buffer[1] = QCFP_ASYNC_DATA_KIN;
		qcfp_format_timestamp(&buffer[2]);

		qcfp_format_float_as_bytes(&buffer[6], get_kinematics_angle(XAXIS));
		qcfp_format_float_as_bytes(&buffer[10], get_kinematics_angle(YAXIS));
		qcfp_format_float_as_bytes(&buffer[14], get_kinematics_angle(ZAXIS));

		qcfp_send_data(buffer, sizeof(buffer));
	}
}

void qcfp_send_filtered_accel(float x, float y, float z)
{
	uint8_t buffer[18];
	if(sensors_get_calibration_state() == SENSORS_CALIBRATED)
	{
		buffer[0] = QCFP_ASYNC_DATA;
		buffer[1] = QCFP_ASYNC_DATA_ACCEL;
		qcfp_format_timestamp(&buffer[2]);

		qcfp_format_float_as_bytes(&buffer[6], x);
		qcfp_format_float_as_bytes(&buffer[10], y);
		qcfp_format_float_as_bytes(&buffer[14], z);

		qcfp_send_data(buffer, sizeof(buffer));
	}
}

void qcfp_send_raw_mag(void)
{
	uint8_t buffer[18];
	if(sensors_get_calibration_state() == SENSORS_CALIBRATED)
	{
		buffer[0] = QCFP_ASYNC_DATA;
		buffer[1] = QCFP_ASYNC_DATA_MAG;
		qcfp_format_timestamp(&buffer[2]);

		qcfp_format_float_as_bytes(&buffer[6], read_compass_raw(XAXIS));
		qcfp_format_float_as_bytes(&buffer[10], read_compass_raw(YAXIS));
		qcfp_format_float_as_bytes(&buffer[14], read_compass_raw(ZAXIS));

		qcfp_send_data(buffer, sizeof(buffer));
	}
}

void qcfp_send_gyro_rate(void)
{
	uint8_t buffer[18];
	if(sensors_get_calibration_state() == SENSORS_CALIBRATED)
	{
		buffer[0] = QCFP_ASYNC_DATA;
		buffer[1] = QCFP_ASYNC_DATA_GYRO;
		qcfp_format_timestamp(&buffer[2]);

		qcfp_format_float_as_bytes(&buffer[6], get_axis_gr(XAXIS));
		qcfp_format_float_as_bytes(&buffer[10], get_axis_gr(YAXIS));
		qcfp_format_float_as_bytes(&buffer[14], get_axis_gr(ZAXIS));

		qcfp_send_data(buffer, sizeof(buffer));
	}
}

void qcfp_send_height_data(uint16_t height)
{
	uint8_t buffer[8];
	buffer[0] = QCFP_ASYNC_DATA;
	buffer[1] = QCFP_ASYNC_DATA_HEIGHT;
	qcfp_format_timestamp(&buffer[2]);
	buffer[6] = (height & 0x000000FF) >> 0;
	buffer[7] = (height & 0x0000FF00) >> 8;
	qcfp_send_data(buffer, sizeof(buffer));
}

static void qcfp_handle_packet(uint8_t packet[], uint8_t length)
{
	uint8_t* payload;
	uint8_t payload_length;
	bool nack = false;

	if(length > 0)
	{
		payload = packet+1;
		payload_length = length-1;

		switch(packet[0])
		{
		case QCFP_SET_THROTTLE:
			nack = qcfp_set_throttle(payload, payload_length);
			break;
		case QCFP_INCREMENT_HEIGHT:
			nack = qcfp_set_increment_height(payload, payload_length);
			break;
		case QCFP_SET_DESIRED_ANGLE:
			nack = qcfp_set_desired_angles(payload, payload_length);
			break;
		case QCFP_CALIBRATE_QUADROTOR:
			nack = qcfp_calibrate_quadrotor_handler(payload, payload_length);
			break;
		case QCFP_FLIGHT_MODE:
			nack = qcfp_flight_mode_handler(payload, payload_length);
			break;
		case QCFP_RAW_MOTOR_CONTROL:
			nack = qcfp_raw_motor_control_handler(payload, payload_length);
			break;
		case QCFP_CONTROL_METHOD_OVERRIDE:
			nack = qcfp_control_method_override_handler(payload, payload_length);
			break;
		default:
			nack = true;
			break;
		}

		if(nack)
		{
			// Format a nack
			response_buffer[0] = QCFP_NACK;
			response_buffer[1] = packet[0];
			response_length = 2;
		}
		else
		{
			response_buffer[0] = packet[0];
		}
		qcfp_send_data(response_buffer, response_length);
	}
}

// ===========================================================================
// 0x24
// ===========================================================================
static bool qcfp_set_throttle(uint8_t payload[], uint8_t length)
{
	bool nack = false;
	uint16_t throttle = 0;
	if(length < 2)
	{
		throttle = read_throttle();
		response_buffer[1] = (throttle & 0x000000FF) >> 0;
		response_buffer[2] = (throttle & 0x0000FF00) >> 8;
		response_length = 3;
	}
	else if((length >= 2) && (control_mode == QCFP_CONTROL_MODE_PID))
	{
		throttle  = (payload[0] << 0) & 0x000000FF;
		throttle |= (payload[1] << 8) & 0x0000FF00;
		write_throttle(throttle);
	}
	return nack;
}

// ===========================================================================
// 0x25
// ===========================================================================
static bool qcfp_set_desired_angles(uint8_t payload[], uint8_t length)
{
	bool nack = false;
	if((length >= 12) && (control_mode == QCFP_CONTROL_MODE_PID))
	{
		float roll = qcfp_format_bytes_as_float(&payload[0]);
		float pitch = qcfp_format_bytes_as_float(&payload[4]);
		float delta_yaw = qcfp_format_bytes_as_float(&payload[8]);
		update_flight_control(roll, pitch, delta_yaw);
	}
	return nack;
}

// ===========================================================================
// 0x26
// ===========================================================================
static bool qcfp_set_increment_height(uint8_t payload[], uint8_t length)
{
	bool nack = false;
	if((length >= 2) && (control_mode == QCFP_CONTROL_MODE_PID))
	{
		int16_t delta_height = payload[0] | (payload[1] << 8);
		set_desired_height_delta(delta_height);
	}
	return nack;
}

// ===========================================================================
// 0x40
// ===========================================================================
#define CMD_40_CALIBRATE_START_STOP_INDEX 0
#define     CMD_40_STOP_CALIBRATION       0
#define     CMD_40_START_CALIBRATION      1
#define     CMD_40_UNABLE_TO_CALIBRATE    3

static bool qcfp_calibrate_quadrotor_handler(uint8_t payload[], uint8_t length)
{
	bool nack = false;
	response_length = 2;

	// 0 length = state query
	if(length == 0)
	{
		response_buffer[1] = sensors_get_calibration_state();
	}
	else if(length == 1)
	{
		if(payload[0] == CMD_40_START_CALIBRATION)
		{
			if(flight_mode == false)
			{
				sensors_set_calibration(true);
				response_buffer[1] = sensors_get_calibration_state();
			}
			else
			{
				// Can't calibrate while flight mode is active
				response_buffer[1] = CMD_40_UNABLE_TO_CALIBRATE;
			}
		}
		else if(payload[0] == CMD_40_STOP_CALIBRATION)
		{
			sensors_set_calibration(false);
			response_buffer[1] = sensors_get_calibration_state();
		}
		else
		{
			nack = true;
		}
	}
	else
	{
		nack = true;
	}
	return nack;
}

// ===========================================================================
// 0x41
// ===========================================================================
#define CMD_41_ENABLE_INDEX 0
#define     CMD_41_DISABLED 0
#define     CMD_41_ENABLED  1
#define     CMD_41_PENDING  2

static void qcfp_flight_mode_handler_esc_enable_timeout(void)
{
	uint8_t buffer[2];
	buffer[0] = QCFP_FLIGHT_MODE;
	buffer[1] = CMD_41_ENABLED;
	flight_mode = true;
	gpio_set_leds(gpio_led_3);
	qcfp_send_data(buffer, sizeof(buffer));
}

static bool qcfp_flight_mode_handler(uint8_t payload[], uint8_t length)
{
	bool nack = false;

	response_length = 2;

	if(length == 0)
	{
		response_buffer[1] = flight_mode;
	}
	else if(length == 1)
	{
		switch(payload[CMD_41_ENABLE_INDEX])
		{
		case CMD_41_ENABLED:
			if(sensors_get_calibration_state() == SENSORS_CALIBRATED)
			{
				if(flight_mode == false)
				{
					pwm_off_all();
					gpio_set_escs(true);
					// Because of the design of the timer module, this post will fail if the callback
					// has already been posted. Because of this, we are guaranteed that the callback
					// is only called once (per enable) since flight mode won't be false after it executes.
					eq_post_timer(qcfp_flight_mode_handler_esc_enable_timeout, ESC_STARTUP_TIME, eq_timer_one_shot);
					// ESCs need more time to initialize
					response_buffer[1] = CMD_41_PENDING;
					gpio_clear_leds(gpio_led_3);
				}
				else
				{
					response_buffer[1] = CMD_41_ENABLED;
				}
			}
			else
			{
				response_buffer[1] = CMD_41_DISABLED;
			}
			break;
		case CMD_41_DISABLED:
			gpio_set_escs(false);
			pwm_off_all();
			response_buffer[1] = CMD_41_DISABLED;
			flight_mode = false;
			gpio_clear_leds(gpio_led_3);
			break;
		default:
			nack = true;
			break;
		}
	}
	else
	{
		nack = true;
	}

	return nack;
}

// ===========================================================================
// 0xF0
// ===========================================================================
#define CMD_F0_PWM_NOT_SET 0
#define CMD_F0_PWM_SET     1

static bool qcfp_raw_motor_control_handler(uint8_t payload[], uint8_t length)
{
	if(length != 4)
	{
		return true;
	}

	if(flight_mode && (control_mode != QCFP_CONTROL_MODE_PID))
	{
		pwm_set(pwm_motor1, payload[0]);
		pwm_set(pwm_motor2, payload[1]);
		pwm_set(pwm_motor3, payload[2]);
		pwm_set(pwm_motor4, payload[3]);
		response_buffer[1] = CMD_F0_PWM_SET;
	}
	else
	{
		response_buffer[1] = CMD_F0_PWM_NOT_SET;
	}

	response_length = 2;
	return false;
}

static bool qcfp_control_method_override_handler(uint8_t payload[], uint8_t length)
{
	if(length == 0)
	{
		response_buffer[1] = control_mode;
	}
	else
	{
		if((payload[0] >= QCFP_CONTROL_MODE_NORMAL) && (payload[0] <= QCFP_CONTROL_MODE_PID))
		{
			control_mode = payload[0];
			response_buffer[1] = payload[0];

			switch(control_mode)
			{
			case QCFP_CONTROL_MODE_NORMAL:
				// Not yet implemented
				break;
			case QCFP_CONTROL_MODE_MANUAL:
				// PID is disabled by the fact that control_mode is not QCFP_CONTROL_MODE_PID
				break;
			case QCFP_CONTROL_MODE_PID:
				pid_init();
				break;
			default:
				break;
			}
		}
	}

	response_length = 2;
	return false;
}
