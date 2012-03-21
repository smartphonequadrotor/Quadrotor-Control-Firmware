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

#include "gpio.h"

#define led_mask (gpio_led_1 | gpio_led_2 | gpio_led_3 | gpio_led_4)

void gpio_set_escs(bool on)
{
	if(on)
	{
		AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA31;
	}
	else
	{
		AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA31;
	}
}

void gpio_set_leds(uint32_t leds)
{
	AT91C_BASE_PIOA->PIO_CODR = (leds & led_mask);
}

void gpio_clear_leds(uint32_t leds)
{
	AT91C_BASE_PIOA->PIO_SODR = (leds & led_mask);
}

void gpio_led_dance(void)
{
	static uint32_t state = true;
	if(state)
	{
		gpio_set_leds(gpio_led_1);
	}
	else
	{
		gpio_clear_leds(gpio_led_1);
	}
	state = !state;
}
