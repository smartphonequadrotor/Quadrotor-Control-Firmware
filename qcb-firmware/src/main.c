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
#include "us0.h"
#include "us1.h"
#include "twi.h"
#include "sensors.h"
#include "eq.h"
#include "qcfp.h"
#include "interrupts.h"
#include "gpio.h"

int main(void)
{
	// Enable peripherals
	system_init();
	pins_init();
	us1_init();
	us0_init();
	twi_init();

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

	while(1)
	{
		eq_dispatch();
		eq_dispatch_timers();
	}
	return 0;
}
