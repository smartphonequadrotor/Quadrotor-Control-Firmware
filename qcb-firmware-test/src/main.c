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

#include "AT91SAM7S161.h"

int main(void)
{
	// Enable clock to Parallel IO Controll A (GPIOs)
	AT91C_BASE_PMC->PMC_PCER = 0x00000004;

	// Configure PWR_CNTL pin (PA31 as output), drive low to enable ESCs
	AT91C_BASE_PIOA->PIO_PER = 0x80000000;
	AT91C_BASE_PIOA->PIO_OER = 0x80000000;

	// Configure BTN3 pin (PA30 as input), keep internal pull enabled,
	// pulled low when pressed
	AT91C_BASE_PIOA->PIO_PER = 0x40000000;

	// Configure LED3 pin (PA19 as output), drive low to turn on
	AT91C_BASE_PIOA->PIO_PER = 0x00080000;
	AT91C_BASE_PIOA->PIO_OER = 0x00080000;

	while(1)
	{
		if(AT91C_BASE_PIOA->PIO_PDSR & 0x40000000)
		{
			AT91C_BASE_PIOA->PIO_SODR = 0x00080000;
			AT91C_BASE_PIOA->PIO_SODR = 0x80000000;
		}
		else
		{
			AT91C_BASE_PIOA->PIO_CODR = 0x00080000;
			AT91C_BASE_PIOA->PIO_CODR = 0x80000000;
		}
	}

	return 42;
}
