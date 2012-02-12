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

#include "us1.h"

static void us1_irq_handler(void)
{
	AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA16;
}

void us1_init(void)
{
	// US1 initialize
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US1);

	// Reset and disable receiver/transmitter
	AT91C_BASE_US1->US_CR = (
			AT91C_US_RSTRX | AT91C_US_RSTTX |
			AT91C_US_RXDIS | AT91C_US_TXDIS
	);

	// Configure USART mode to be asynchronous 8 bits, no parity, 1 stop
	AT91C_BASE_US1->US_MR = (
			AT91C_US_USMODE_HWHSH | // Asynchronous hardware handshaking mode
			AT91C_US_CLKS_CLOCK | // Use MCK
			AT91C_US_CHRL_8_BITS |
			AT91C_US_PAR_NONE | // No parity
			AT91C_US_NBSTOP_1_BIT | // 1 stop bit
			AT91C_US_OVER
	);

	// Configure the baudrate
	// PLL clock is clocked at 18.432*73/14
	// Main clock is clocked at PLL clock/2
	// MCK = 48 054 857.14 Hz
	// MCK/(8*(2-OVER)*(CD+FP/8)) = 115239.4656
	// Where CD = 52, FP = 1, OVER = 1
	// Percent error is then 0.20787%
	AT91C_BASE_US1->US_BRGR = (
			((52 & 0xFFFF) << 0 ) |
			((1  & 0x0007) << 16)
	);

	// Configure US1 in AIC
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_US1] = (uint32_t)us1_irq_handler;
	AT91C_BASE_AIC->AIC_SMR[AT91C_ID_US1] = 0;
	AT91C_BASE_AIC->AIC_IECR = (1 << AT91C_ID_US1);

	// Enable peripheral interrupts
	AT91C_BASE_US1->US_IER = AT91C_US_RXRDY | AT91C_US_TXRDY;

	// Enable both the receiver
	// The transmitter isn't enabled until we have something to transmit
	AT91C_BASE_US1->US_CR = AT91C_US_RXEN;
}
