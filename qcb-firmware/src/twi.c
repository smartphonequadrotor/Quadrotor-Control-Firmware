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

#include "twi.h"
#include "pins.h"

// http://www.i2c-bus.org/highspeed/
// HIGH to LOW ratio of 1 to 2
// These values are for 400KHz clock
// ((1/400 000) * (1/3) * 48 054 857 - 4 = 36.0457 = CHDIV*2^CKDIV = 9*2^2
#define TWI_CLOCK_HIGH_DIV  9
// ((1/400 000) * (2/3) * 48 054 857 - 4 = 76.0914 = CLDIV*2^CKDIV = 19*2^2
#define TWI_CLOCK_LOW_DIV   19

#define TWI_CLOCK_DIV       2

void twi_init(void)
{
	// Clock the TWI peripheral
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TWI);

	// Reset the TWI
	AT91C_BASE_TWI->TWI_CR = AT91C_TWI_SWRST;
	//AT91C_BASE_TWI->TWI_RHR;

	// Disable master and slave
	//AT91C_BASE_TWI->TWI_CR = AT91C_TWI_MSDIS;

	// Set master mode
	AT91C_BASE_TWI->TWI_CR = AT91C_TWI_MSEN;

	// Configure clock
	//	AT91C_BASE_TWI.TWI_CWGR = 0;
	AT91C_BASE_TWI->TWI_CWGR = (
			(TWI_CLOCK_DIV << 16) |
			(TWI_CLOCK_LOW_DIV << 8) |
			TWI_CLOCK_HIGH_DIV
	);
}
