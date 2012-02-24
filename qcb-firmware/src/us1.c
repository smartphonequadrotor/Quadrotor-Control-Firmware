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
#include "pins.h"
#include "qcfp.h"
#include "eq.h"

// Defines
#define US1_RX_TIMEOUT 100 // In bit periods, this corresponds to 10 characters
#define US1_INTERRUPT_PRIORITY 0 // No nested interrupts so all priorities should be 0
// PLL clock is clocked at 18.432*73/14
// Main clock is clocked at PLL clock/2
// MCK = 48 054 857.14 Hz
// MCK/(8*(2-OVER)*(CD+FP/8)) = 115239.4656
// Where CD = 52, FP = 1, OVER = 1
// Percent error is then 0.20787%
#define US1_BAUD_RATE_REG_VALUE (((52 & 0xFFFF) << 0 ) | ((1  & 0x0007) << 16))

// Static functions
static void us1_irq_handler(void);

// Static variables
// Ping-pong rx buffers
static uint8_t rx_buffer_1[MAX_QCFP_PACKET_SIZE];
static uint8_t rx_buffer_2[MAX_QCFP_PACKET_SIZE];
// Transmit circular buffer
//static circular_buffer_t tx_buffer;

static void us1_irq_handler(void)
{
	uint32_t us1_status = AT91C_BASE_US1->US_CSR;

	// PDC buffer has been emptied
	if(us1_status & AT91C_US_ENDTX)
	{
		// Check if there is more data to send

		// Set up next tx buffer

		// If no more data, disable transmitter and ENDTX interrupts

	}

	// Rx line has been inactive or PDC buffer has been filled
	if((us1_status & AT91C_US_TIMEOUT) || (us1_status & AT91C_US_ENDRX))
	{
		if(us1_status & AT91C_US_TIMEOUT)
		{
			// Start another timeout after more data is received
			AT91C_BASE_US1->US_CR = AT91C_US_STTTO;
		}

		// Note about ENDRX:
		// This interrupt should be rare, our ping-pong buffers should be
		// large enough that an entire packet is received before they are
		// filled.

		// Swap receive buffers
		if(AT91C_BASE_PDC_US1->PDC_RPR == (uint32_t)rx_buffer_1)
		{
			AT91C_BASE_PDC_US1->PDC_RPR = (uint32_t)rx_buffer_2;
			// Post event to handle received data
			eq_post(
					qcfp_data_received,
					rx_buffer_1,
					MAX_QCFP_PACKET_SIZE - AT91C_BASE_PDC_US1->PDC_RCR
			);
		}
		else
		{
			// Swap buffers
			AT91C_BASE_PDC_US1->PDC_RPR = (uint32_t)rx_buffer_1;
			// Post event to handle received data
			eq_post(
					qcfp_data_received,
					rx_buffer_2,
					MAX_QCFP_PACKET_SIZE - AT91C_BASE_PDC_US1->PDC_RCR
			);
		}
		// Reset number of bytes available in buffer
		AT91C_BASE_PDC_US1->PDC_RCR = MAX_QCFP_PACKET_SIZE;
	}
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
	AT91C_BASE_US1->US_BRGR = US1_BAUD_RATE_REG_VALUE;

	// Configure the receive timeout and have it start after a receive starts
	AT91C_BASE_US1->US_RTOR = US1_RX_TIMEOUT;
	AT91C_BASE_US1->US_CR = AT91C_US_STTTO;

	// Configure the US1 DMA controller
	// Setting an rx buffer clears the endrx flag
	AT91C_BASE_PDC_US1->PDC_RPR = (uint32_t)rx_buffer_1;
	AT91C_BASE_PDC_US1->PDC_RCR = MAX_QCFP_PACKET_SIZE;
	// Enable PDC transfers
	// Rx and tx are enabled. In the case of tx, no transfers will occur
	// until a pointer is set and the US1 tx interrupts enabled
	AT91C_BASE_PDC_US1->PDC_PTCR = AT91C_PDC_RXTEN | AT91C_PDC_TXTEN;

	// Configure US1 in AIC
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_US1] = (uint32_t)us1_irq_handler;
	AT91C_BASE_AIC->AIC_SMR[AT91C_ID_US1] = US1_INTERRUPT_PRIORITY;
	AT91C_BASE_AIC->AIC_IECR = (1 << AT91C_ID_US1);

	// Enable peripheral interrupts
	AT91C_BASE_US1->US_IER = AT91C_US_ENDRX | AT91C_US_TIMEOUT;

	// Enable the receiver
	// The transmitter isn't enabled until we have something to transmit
	AT91C_BASE_US1->US_CR = AT91C_US_RXEN;
}

void us1_send_buffer(uint32_t buffer[], uint16_t length)
{

}
