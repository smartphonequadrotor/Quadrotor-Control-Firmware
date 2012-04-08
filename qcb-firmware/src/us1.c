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
#include "circular_buffer.h"
#include "interrupts.h"

// Defines
#define US1_RX_TIMEOUT 100 // In bit periods, this corresponds to 10 byte times
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
static void us1_kickstart_tx(void);
static void us1_setup_tx_buffer(void);

// Static variables
// Ping-pong rx buffers
static uint8_t rx_buffer_1[QCFP_MAX_PACKET_SIZE];
static uint8_t rx_buffer_2[QCFP_MAX_PACKET_SIZE];
static uint8_t* current_buffer = rx_buffer_1;

// Transmit circular buffer
#define TX_BUFFER_LENGTH 12*QCFP_MAX_PACKET_SIZE
static circular_buffer_t tx_circular_buffer;
static uint8_t tx_buffer[TX_BUFFER_LENGTH];

// Indicates whether a transmit is currently in progress
static bool tx_in_progress = false;
// Counts the last number of bytes sent through the PDC interface
static uint16_t last_num_bytes_sent;

static void us1_irq_handler(void)
{
	uint32_t us1_status = AT91C_BASE_US1->US_CSR;

	// PDC buffer has been emptied
	if(us1_status & AT91C_US_ENDTX)
	{
		// Transmit completed, release the part of the buffer that we using
		cb_remove_bytes(&tx_circular_buffer, last_num_bytes_sent);
		last_num_bytes_sent = 0;

		// Check if there is more data to send
		if(tx_circular_buffer.size > 0)
		{
			us1_setup_tx_buffer();
		}
		else
		{
			// No more data, disable transmitter and ENDTX interrupts
			AT91C_BASE_PDC_US1->PDC_TCR = 0;
			AT91C_BASE_US1->US_IDR = AT91C_US_ENDTX;
			tx_in_progress = false;
		}
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
		if(current_buffer == rx_buffer_1)
		{
			// Swap buffers
			current_buffer = rx_buffer_2;
			AT91C_BASE_PDC_US1->PDC_RPR = (uint32_t)rx_buffer_2;
			// Post event to handle received data
			eq_post(
					qcfp_data_received,
					rx_buffer_1,
					QCFP_MAX_PACKET_SIZE - AT91C_BASE_PDC_US1->PDC_RCR
			);
		}
		else
		{
			// Swap buffers
			current_buffer = rx_buffer_1;
			AT91C_BASE_PDC_US1->PDC_RPR = (uint32_t)rx_buffer_1;
			// Post event to handle received data
			eq_post(
					qcfp_data_received,
					rx_buffer_2,
					QCFP_MAX_PACKET_SIZE - AT91C_BASE_PDC_US1->PDC_RCR
			);
		}
		// Reset number of bytes available in buffer
		AT91C_BASE_PDC_US1->PDC_RCR = QCFP_MAX_PACKET_SIZE;
	}
}

void us1_init(void)
{
	// Initialize the transmit buffer
	cb_init(&tx_circular_buffer, tx_buffer, TX_BUFFER_LENGTH);

	// Initialize US1 peripheral
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
	AT91C_BASE_PDC_US1->PDC_RCR = QCFP_MAX_PACKET_SIZE;
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
	AT91C_BASE_US1->US_CR = AT91C_US_RXEN | AT91C_US_TXEN;
}

/*
 * Adds length bytes from buffer to the transmit circular buffer associated
 * with the US1 interface. This function relies on the fact that it is the
 * only function to *add* bytes to the buffer. Which this function is in
 * progress, bytes may be removed, but not added. This function does not
 * update the size field of the buffer until after all bytes have been added
 * to minimize the number of times interrupts must be disabled.
 */
us1_error_t us1_send_buffer(uint8_t buffer[], uint16_t length)
{
	us1_error_t ret_val = US1_SUCCESS;
	uint16_t bytes_added = 0;

	if((tx_circular_buffer.size + length) < tx_circular_buffer.capacity)
	{
		while(bytes_added != length)
		{
			cb_add_byte(&tx_circular_buffer, buffer[bytes_added++]);
		}
		// Update the number of bytes in the circular buffer
		interrupts_disable();
		tx_circular_buffer.size += length;
		us1_kickstart_tx();
		interrupts_enable();
	}
	else
	{
		ret_val = US1_NOT_ENOUGH_ROOM;
	}
	return ret_val;
}

/*
 * Starts a transmit on the peripheral if one is not already in progress.
 * Requires protection from a transmit complete interrupt occurring at the
 * same time.
 */
void us1_kickstart_tx(void)
{
	if(!tx_in_progress)
	{
		tx_in_progress = true;
		last_num_bytes_sent = 0;
		// Enable tx buffer empty interrupts
		AT91C_BASE_US1->US_IER = AT91C_US_ENDTX;
	}
}

/*
 * Requires there to be data to the transmitted in the circular buffer.
 */
void us1_setup_tx_buffer(void)
{
	// Set up next tx buffer
	AT91C_BASE_PDC_US1->PDC_TPR = (uint32_t)&(tx_circular_buffer.data[tx_circular_buffer.read_index]);
	// Determine the number of bytes to send since only contiguous bytes can be sent
	if((tx_circular_buffer.capacity - tx_circular_buffer.read_index) < tx_circular_buffer.size)
	{
		AT91C_BASE_PDC_US1->PDC_TCR = tx_circular_buffer.capacity - tx_circular_buffer.read_index;
		last_num_bytes_sent = tx_circular_buffer.capacity - tx_circular_buffer.read_index;
	}
	else
	{
		AT91C_BASE_PDC_US1->PDC_TCR = tx_circular_buffer.size;
		last_num_bytes_sent = tx_circular_buffer.size;
	}
}
