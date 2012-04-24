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

#include "us0.h"
#include "us1.h"
#include "eq.h"
#include "qcfp.h"
#include "pid/flight_controller.h"

// Defines
#define US0_RX_TIMEOUT 100 // In bit periods, this corresponds to 10 byte times
#define US0_INTERRUPT_PRIORITY 0 // No nested interrupts so all priorities should be 0
// PLL clock is clocked at 18.432*73/14
// Main clock is clocked at PLL clock/2
// MCK = 48 054 857.14 Hz
// MCK/(8*(2-OVER)*(CD+FP/8)) = 9599.5
// Where CD = 312, FP = 7, OVER = 0
// Percent error is then 0.20787%
#define US0_BAUD_RATE_REG_VALUE (((312 & 0xFFFF) << 0 ) | ((7  & 0x0007) << 16))
#define US0_BUFFER_SIZE 6

// Static functions
static void us0_irq_handler(void);
static void us0_height_data_received(uint8_t buffer[], uint8_t buffer_size);

// Static variables
static uint8_t rx_buffer[US0_BUFFER_SIZE];

static void us0_irq_handler(void)
{
	uint32_t us0_status = AT91C_BASE_US0->US_CSR;

	// Rx line has been inactive or PDC buffer has been filled
	if(us0_status & AT91C_US_TIMEOUT)
	{
		// Start another timeout after more data is received
		AT91C_BASE_US0->US_CR = AT91C_US_STTTO;

		// Post event to handle received data
		eq_post(
				us0_height_data_received,
				rx_buffer,
				US0_BUFFER_SIZE - AT91C_BASE_PDC_US0->PDC_RCR
		);

		// Reset number of bytes available in buffer
		AT91C_BASE_PDC_US0->PDC_RPR = (uint32_t)rx_buffer;
		AT91C_BASE_PDC_US0->PDC_RCR = US0_BUFFER_SIZE;
	}

	// This is not expected because the buffer size is large enough for
	// an entire height "packet"
	if(us0_status & AT91C_US_ENDRX)
	{
		// Reset number of bytes available in buffer
		AT91C_BASE_PDC_US0->PDC_RPR = (uint32_t)rx_buffer;
		AT91C_BASE_PDC_US0->PDC_RCR = US0_BUFFER_SIZE;
	}
}

void us0_init(void)
{
	// Initialize US0 peripheral
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US0);

	// Reset and disable receiver/transmitter
	AT91C_BASE_US0->US_CR = (
			AT91C_US_RSTRX | AT91C_US_RSTTX |
			AT91C_US_RXDIS | AT91C_US_TXDIS
	);

	// Configure USART mode to be asynchronous 8 bits, no parity, 1 stop
	AT91C_BASE_US0->US_MR = (
			AT91C_US_USMODE_HWHSH | // Asynchronous hardware handshaking mode
			AT91C_US_CLKS_CLOCK | // Use MCK
			AT91C_US_CHRL_8_BITS |
			AT91C_US_PAR_NONE | // No parity
			AT91C_US_NBSTOP_1_BIT // 1 stop bit
	);

	// Configure the baudrate
	AT91C_BASE_US0->US_BRGR = US0_BAUD_RATE_REG_VALUE;

	// Configure the receive timeout and have it start after a receive starts
	AT91C_BASE_US0->US_RTOR = US0_RX_TIMEOUT;
	AT91C_BASE_US0->US_CR = AT91C_US_STTTO;

	// Configure the US0 DMA controller
	// Setting an rx buffer clears the endrx flag
	AT91C_BASE_PDC_US0->PDC_RPR = (uint32_t)rx_buffer;
	AT91C_BASE_PDC_US0->PDC_RCR = US0_BUFFER_SIZE;
	// Enable PDC transfers
	// Rx and tx are enabled. In the case of tx, no transfers will occur
	// until a pointer is set and the US0 tx interrupts enabled
	AT91C_BASE_PDC_US0->PDC_PTCR = AT91C_PDC_RXTEN;

	// Configure US0 in AIC
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_US0] = (uint32_t)us0_irq_handler;
	AT91C_BASE_AIC->AIC_SMR[AT91C_ID_US0] = US0_INTERRUPT_PRIORITY;
	AT91C_BASE_AIC->AIC_IECR = (1 << AT91C_ID_US0);

	// Enable peripheral interrupts
	AT91C_BASE_US0->US_IER = AT91C_US_ENDRX | AT91C_US_TIMEOUT;

	// Enable the receiver
	// The transmitter isn't enabled until we have something to transmit
	AT91C_BASE_US0->US_CR = AT91C_US_RXEN;
}
uint8_t patch[10];
uint8_t patch_size = 0;

void us0_height_data_received(uint8_t buffer[], uint8_t buffer_size)
{
	uint8_t n1, n2, n3;
	uint16_t height;

	//add to patch.
	for(int i = 0; i < buffer_size; i++){
		patch[patch_size+i] = buffer[i];
	}

	patch_size += buffer_size;

	while((patch_size >= 5) && (patch[0] == 'R') && (patch[4] == '\r'))
	{
		n1 = patch[1];
		n2 = patch[2];
		n3 = patch[3];
		if((n1 >= '0') && (n1 <= '9') && (n2 >= '0') && (n2 <= '9') && (n3 >= '0') && (n3 <= '9'))
		{
			height = (n1 - '0')*100 + (n2 - '0')*10 + (n3 - '0');
			qcfp_send_height_data(height);
			set_sensor_height(height);
		}
		patch_size -= 5;

		for(int i = 0; i < patch_size ; i++){
			patch[i] = patch[i+5];
		}

	}
}
