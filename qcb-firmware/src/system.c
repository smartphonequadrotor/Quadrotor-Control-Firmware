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

#include "system.h"
#include "pins.h"

// Defines
#define SYS_INTERRUPT_PRIORITY 0 // No nested interrupts so all priorities should be 0
#define PIT_PERIOD             3002 // MCK/16 = 3003428.571 Hz therefore this is ~1ms

// Static functions
static void system_irq_handler(void);

// Static variables
static uint32_t uptime; // System uptime in ms

// Implementation

/*
 * Initializes the system peripherals - RSTC, RTT, PIT, WDT, MC - if their
 * behaviour is different than when configured in startup.s.
 */
void system_init(void)
{
	// Configure the system interrupt
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_SYS] = (uint32_t)system_irq_handler;
	AT91C_BASE_AIC->AIC_SMR[AT91C_ID_SYS] = SYS_INTERRUPT_PRIORITY;

	// Configure the PIT
	AT91C_BASE_PITC->PITC_PIMR  = PIT_PERIOD;// Configure for 1 ms period
	AT91C_BASE_PITC->PITC_PIMR |= AT91C_PITC_PITEN; // Enable counter
	AT91C_BASE_PITC->PITC_PIMR |= AT91C_PITC_PITIEN; // Enable interrupt

	// Enable the system interrupt
	AT91C_BASE_AIC->AIC_IECR = (1 << AT91C_ID_SYS);
}

uint32_t system_uptime(void)
{
	return uptime;
}

static void system_irq_handler(void)
{
	uint32_t num_ticks_since_last_read;

	// Handle PIT interrupt
	if(AT91C_BASE_PITC->PITC_PISR & AT91C_PITC_PITS)
	{
		// Read PIT_PIVR register to clear the interrupt
		num_ticks_since_last_read = AT91C_BASE_PITC->PITC_PIVR >> 20;
		uptime += num_ticks_since_last_read;
	}
}
