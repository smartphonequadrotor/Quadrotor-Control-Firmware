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
#include "eq.h"
#include "pins.h"
#include "at91lib/twi.h"
#include "at91lib/twid.h"

// http://www.i2c-bus.org/highspeed/
// HIGH to LOW ratio of 1 to 2
// These values are for 400KHz clock
// ((1/400 000) * (1/3) * 48 054 857 - 4 = 36.0457 = CHDIV*2^CKDIV = 9*2^2
//#define TWI_CLOCK_HIGH_DIV  9
// ((1/400 000) * (2/3) * 48 054 857 - 4 = 76.0914 = CLDIV*2^CKDIV = 19*2^2
//#define TWI_CLOCK_LOW_DIV   19
//#define TWI_CLOCK_DIV       2

#define TWI_CLOCK_RATE                   400000
#define TWI_INTERRUPT_PRIORITY           0
#define TWI_MAX_OUTSTANDING_TRANSACTIONS 20
// Corresponds to the max size of the buffers in the event queue
#define TWI_MAX_RX_SIZE                  32

typedef enum twi_mode
{
	twi_read,
	twi_write,
} twi_mode;

// File local functions
static void twi_irq_handler(void);
static void twi_async_callback(Async* a);
static void twi_add_transaction(uint8_t address, eq_handler cb, uint8_t reg, uint8_t value, uint8_t length, twi_mode mode);
static void twi_kickstart(void);
static void twi_process_next_transfer(uint8_t unused1[], uint8_t unused2);

// Driver structure
static Twid twi;
// Async structure. Invokes default callback upon completion to advance state machine
static Async twi_async = {0, twi_async_callback, {0, 0, 0, 0}};
static uint8_t twi_buffer[TWI_MAX_RX_SIZE]; // Used for reads. After a read completes, data is posted to the eq

/*
 * Set to false in the interrupt so the next time the twi_process_next_transfer
 * event is processed a new transfer starts.
 * Checked when a transfer is posted: if false, starts a new transfer, if
 * true, lets the interrupt handle post an event that will kickoff a transfer
 */
static bool twi_transfer_in_progress = false;

typedef struct twi_transaction
{
	uint8_t address;
	uint8_t data[2]; // Used for writes
	uint8_t length;
	twi_mode mode;
	eq_handler callback;
} twi_transaction;

typedef struct twi_transaction_queue
{
	uint8_t read_index;
	uint8_t write_index;
	uint8_t size;
	twi_transaction transactions[TWI_MAX_OUTSTANDING_TRANSACTIONS];
} twi_transaction_queue;

// Queues pending transactions
static twi_transaction_queue twi_events;

// The currently executing transaction
static twi_transaction current_transaction;

// Initializes transaction circular buffer, peripheral, driver, and interrupts
void twi_init(void)
{
	twi_events.read_index = 0;
	twi_events.write_index = 0;
	twi_events.size = 0;

	// Clock the TWI peripheral
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TWI);

	// Configure the peripheral
	TWI_ConfigureMaster(AT91C_BASE_TWI, TWI_CLOCK_RATE, QCB_MCK);

	// Initialize the driver
	TWID_Initialize(&twi, AT91C_BASE_TWI);

	// Configure TWI in AIC
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_TWI] = (uint32_t)twi_irq_handler;
	AT91C_BASE_AIC->AIC_SMR[AT91C_ID_TWI] = TWI_INTERRUPT_PRIORITY;
	AT91C_BASE_AIC->AIC_IECR = (1 << AT91C_ID_TWI);
}

void twi_write_register(uint8_t address, uint8_t reg, uint8_t value, eq_handler cb)
{
	if(twi_events.size < TWI_MAX_OUTSTANDING_TRANSACTIONS)
	{
		twi_add_transaction(address, cb, reg, value, 2, twi_write);
		twi_kickstart();
	}
}

void twi_read_register(uint8_t address, uint8_t reg, uint8_t num_bytes, eq_handler cb)
{
	if(num_bytes > TWI_MAX_RX_SIZE)
	{
		return;
	}

	// size+1 since we need space for two transactions, a register write and a read
	if((twi_events.size + 1) < TWI_MAX_OUTSTANDING_TRANSACTIONS)
	{
		// Add write register
		twi_add_transaction(address, NULL, reg, 0, 1, twi_write);
		// Add read registers
		twi_add_transaction(address, cb, 0, 0, num_bytes, twi_read);
		// Start transfers if required
		twi_kickstart();
	}
}

// IRQ handler. Handled by driver interface
static void twi_irq_handler(void)
{
	TWID_Handler(&twi);
}

// Will execute in the TWI interrupt
static void twi_async_callback(Async* unused)
{
	if(current_transaction.callback)
	{
		if(current_transaction.mode == twi_read)
		{
			eq_post(
					current_transaction.callback,
					twi_buffer,
					current_transaction.length
					);
		}
		else
		{
			eq_post(
					current_transaction.callback,
					NULL,
					0
					);
		}
	}

	if(twi_events.size > 0)
	{
		eq_post(twi_process_next_transfer, NULL, 0);
	}

	twi_transfer_in_progress = false;
}

static void twi_add_transaction(uint8_t address, eq_handler cb, uint8_t reg, uint8_t value, uint8_t length, twi_mode mode)
{
	twi_transaction new_transaction;
	new_transaction.address = address;
	new_transaction.callback = cb;
	new_transaction.data[0] = reg;
	new_transaction.data[1] = value;
	new_transaction.length = length;
	new_transaction.mode = mode;
	twi_events.transactions[twi_events.write_index++] = new_transaction;
	if(twi_events.write_index >= TWI_MAX_OUTSTANDING_TRANSACTIONS)
	{
		twi_events.write_index = 0;
	}
	twi_events.size++;
}

// Must be called from main loop. Initializes a transfer if one is not
// already in progress.
static void twi_kickstart(void)
{
	twi_process_next_transfer(0, 0);
}

// Must be called from the main loop.
static void twi_process_next_transfer(uint8_t unused1[], uint8_t unused2)
{
	if(twi_transfer_in_progress == false)
	{
		if(twi_events.size > 0)
		{
			// Start transfer
			// If twi_transfer_in_progress is false, it can not be set to true
			// by anything except this function
			twi_transfer_in_progress = true;

			// Pop an event
			current_transaction = twi_events.transactions[twi_events.read_index++];
			if(twi_events.read_index >= TWI_MAX_OUTSTANDING_TRANSACTIONS)
			{
				twi_events.read_index = 0;
			}
			twi_events.size--;
			if(current_transaction.mode == twi_read)
			{
				TWID_Read(
						&twi,
						current_transaction.address,
						0,
						0,
						twi_buffer,
						current_transaction.length,
						&twi_async
						);
			}
			else
			{
				TWID_Write(
						&twi,
						current_transaction.address,
						0,
						0,
						current_transaction.data,
						current_transaction.length,
						&twi_async
						);
			}
		}
	}
}
