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

/* AT91SAM7S161.h doesn't define all pins, so we define them here */
#define PA_0  (1 << 0)
#define PA_1  (1 << 1)
#define PA_2  (1 << 2)
#define PA_3  (1 << 3)
#define PA_4  (1 << 4)
#define PA_5  (1 << 5)
#define PA_6  (1 << 6)
#define PA_7  (1 << 7)
#define PA_8  (1 << 8)
#define PA_9  (1 << 9)
#define PA_10 (1 << 10)
#define PA_11 (1 << 11)
#define PA_12 (1 << 12)
#define PA_13 (1 << 13)
#define PA_14 (1 << 14)
#define PA_15 (1 << 15)
#define PA_16 (1 << 16)
#define PA_17 (1 << 17)
#define PA_18 (1 << 18)
#define PA_19 (1 << 19)
#define PA_20 (1 << 20)
#define PA_21 (1 << 21)
#define PA_22 (1 << 22)
#define PA_23 (1 << 23)
#define PA_24 (1 << 24)
#define PA_25 (1 << 25)
#define PA_26 (1 << 26)
#define PA_27 (1 << 27)
#define PA_28 (1 << 28)
#define PA_29 (1 << 29)
#define PA_30 (1 << 30)
#define PA_31 (1 << 31)

#define PID_2  (1 << 2)  // PIOA  - Parallel I/O Controller A
#define PID_7  (1 << 7)  // US1   - USART1

// Unbelievably, the Atmel header for this part does not include a define
// for the base address of the USART1 peripheral
#define AT91C_BASE_US1    (AT91_CAST(AT91PS_USART)0xFFFC4000)

// Test firmware defines
#define CMD_RECEIVING    0
#define CMD_TRANSMITTING 1

int main(void)
{
	uint8_t u8_rx_char = 0;
	uint8_t u8_cmd_state = CMD_RECEIVING;

	//------------------------------------------------------------------------
	// Configure GPIO pins
	//------------------------------------------------------------------------

	// Enable clock to Parallel IO Control A (GPIOs)
	AT91C_BASE_PMC->PMC_PCER = PID_2;

	// Configure PWR_CNTL pin (PA31 as output), drive low to enable ESCs
	AT91C_BASE_PIOA->PIO_PER = PA_31;
	AT91C_BASE_PIOA->PIO_OER = PA_31;

	// Configure BTN3 pin as an input (default), keep internal pull
	// enabled (default)
	// Note: Line is pulled low when pressed
	AT91C_BASE_PIOA->PIO_PER = PA_30;

	// Configure LED3 pin (PA19 as output), drive low to turn on
	AT91C_BASE_PIOA->PIO_PER = PA_19;
	AT91C_BASE_PIOA->PIO_OER = PA_19;

	//------------------------------------------------------------------------
	// Configure USART1
	//------------------------------------------------------------------------

	// Enable clock to USART1
	AT91C_BASE_PMC->PMC_PCER = PID_7;

	// Select the peripheral function for pins 21, 22, 24, 25
	// Peripheral A is the default for these pins after reset
	// Disable PIO control of pins 21, 22, 24, 25 so the peripheral use them
	AT91C_BASE_PIOA->PIO_PDR = (PA_21 | PA_22 | PA_24 | PA_25);

	// Configure USART mode to be asynchronous 8 bits, no parity, 1 stop
	AT91C_BASE_US1->US_MR = (
			AT91C_US_USMODE_NORMAL | // Not RS485 or IrDA
			AT91C_US_CLKS_CLOCK | // Use MCK
			AT91C_US_CHRL_8_BITS |
			AT91C_US_PAR_NONE | // No parity
			AT91C_US_NBSTOP_1_BIT // 1 stop bit
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
	// Set OVER bit
	AT91C_BASE_US1->US_MR |= AT91C_US_OVER;

	// Enable both the transmitter and receiver
	AT91C_BASE_US1->US_CR = AT91C_US_TXEN | AT91C_US_RXEN;

	while(1)
	{
		// Turns on power to the ESCs and LED 4 when button 4 is pressed
		if(AT91C_BASE_PIOA->PIO_PDSR & PA_30)
		{
			AT91C_BASE_PIOA->PIO_SODR = PA_19;
			AT91C_BASE_PIOA->PIO_SODR = PA_31;
		}
		else
		{
			AT91C_BASE_PIOA->PIO_CODR = PA_19;
			AT91C_BASE_PIOA->PIO_CODR = PA_31;
		}

		// Receive data over uart and set pins appropriately

		if(u8_cmd_state == CMD_RECEIVING)
		{
			// Wait for character to be received
			while ((AT91C_BASE_US1->US_CSR & AT91C_US_RXRDY) == 0);
			// Get received character
			u8_rx_char = AT91C_BASE_US1->US_RHR;
		}
		else if(u8_cmd_state == CMD_TRANSMITTING)
		{
			// Make sure nothing is being transmitted
			while((AT91C_BASE_US1->US_CSR & AT91C_US_TXEMPTY) == 0);

			// Write byte to transmit holding register
			AT91C_BASE_US1->US_THR = 0x42;

			// Wait for byte to be sent
			while((AT91C_BASE_US1->US_CSR & AT91C_US_TXEMPTY) == 0);
		}
		else
		{
			while(1); // Unrecognized state
		}

	}

	return 42;
}
