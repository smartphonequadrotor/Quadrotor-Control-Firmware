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

#include "qcb.h"
#include "system.h"
#include "pins.h"
#include "us1.h"
#include "twi.h"
#include "sensors.h"
#include "eq.h"
#include "qcfp.h"
#include "interrupts.h"
#include "gpio.h"
#include "pwm.h"

int main(void)
{
	// Enable peripherals
	system_init();
	pins_init();
	us1_init();
	twi_init();
	pwm_init();

	// Initialize state machines
	qcfp_init();
	eq_init();

	// Once everything is initialized, enable interrupts globally
	interrupts_enable();

	// Enable Expansion module 3 and 4 (sensors plugged into these temporarily)
	AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA20;
	AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA23;

	sensors_init();

	eq_post_timer(gpio_led_dance, 250, eq_timer_periodic);

	//******************INIT**********************

	//_X_ we want to init our sensors...
	//we want to calibrate sensors and put system in "good" state.
	//we want to init motors to default (0) values.
	//_X_ we need to init kinematics to 0 values.
	//_X_ we need to init windup guard values for PID extreme conditions.
	//we MAY need to init rangefinder for height sensing.
	//_X_we need to set up the fourth order filter for accelerometers.

	//******************TASKS**********************
	//4kHz:
	//		measure Accelerometer, Gyroscope.
	//100Hz:
	//		update GD_t (SECONDS as a float between 100Hz updates )
	//		evaluate m/s
	//		evaluate gyro rate
	//		apply 4th order filter on accel values
	//		calculate kinematics using gyro and accel
	//		*****PID ONLY*****
	//			__?Adjust throttle using height
	//			process flight control
	//50Hz:
	//		--read pilot commands
	//		get height from rangefinder
	//10Hz:
	//		magnetometer heading update using kinematics data

	while(1)
	{
		eq_dispatch();
		eq_dispatch_timers();
	}
	return 0;
}
