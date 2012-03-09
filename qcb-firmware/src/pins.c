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

#include "pins.h"

#define GPIO_INPUTS                ( \
									AT91C_PIO_PA26 | /*NC*/ \
									AT91C_PIO_PA27 | AT91C_PIO_PA28 | AT91C_PIO_PA29 | AT91C_PIO_PA30 /*BTN0,BTN1,BTN2,BTN3*/ \
									)

#define GPIO_OUTPUTS               ( \
									AT91C_PIO_PA8  | /*BT_EN*/ \
									AT91C_PIO_PA10 | AT91C_PIO_PA15 | AT91C_PIO_PA20 | AT91C_PIO_PA23 | /*EN_1,EN_2,EN_3,EN_4*/ \
									AT91C_PIO_PA16 | AT91C_PIO_PA17 | AT91C_PIO_PA18 | AT91C_PIO_PA19 | /*LED0,LED1,LED2,LED3*/ \
									AT91C_PIO_PA31 /*PWR_CNTL (to electronic speed controls)*/ \
									)

#define GPIO_OUTPUT_DEFAULT_LOW    ( 0 \
									)

#define GPIO_OUTPUT_DEFAULT_HIGH   ( \
									AT91C_PIO_PA8  | /*BT on*/ \
									AT91C_PIO_PA10 | AT91C_PIO_PA15 | AT91C_PIO_PA20 | AT91C_PIO_PA23 | /*Expansion modules off*/ \
									AT91C_PIO_PA16 | AT91C_PIO_PA17 | AT91C_PIO_PA18 | AT91C_PIO_PA19 | /*LEDs off*/ \
									AT91C_PIO_PA31 /*ESCs should be powered off until everything (phone and qcb) is ready*/ \
									)

#define PERIPHERAL_A_FUNCTION      ( \
									AT91C_PIO_PA0  | AT91C_PIO_PA1  | AT91C_PIO_PA2 | /*PWM0,PWM1,PWM2*/ \
									AT91C_PIO_PA3  | AT91C_PIO_PA4  | /*SDL, SCL*/  \
									AT91C_PIO_PA5  | AT91C_PIO_PA6  | /*RXD0,TXD0*/ \
									AT91C_PIO_PA11 | AT91C_PIO_PA12 | AT91C_PIO_PA13 | AT91C_PIO_PA14 | /*CS0,MISO,MOSI,SPCK*/ \
									AT91C_PIO_PA21 | AT91C_PIO_PA22 | /*RXD0,TXD0*/ \
									AT91C_PIO_PA24 | AT91C_PIO_PA25   /*RTS1,CTS1*/ \
									)

#define PERIPHERAL_B_FUNCTION      ( \
									AT91C_PIO_PA7  | /*PWM3*/ \
									AT91C_PIO_PA9    /*CS1*/  \
									)

/*
 * Initialize the pins.
 */
void pins_init(void)
{
	// Enable clock to Parallel IO Control A (GPIOs)
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_PIOA);

	// Make sure all pins default to inputs under PIO control with pull ups
	AT91C_BASE_PIOA->PIO_ODR = 0xffffffff;
	AT91C_BASE_PIOA->PIO_PPUER = 0xffffffff;
	AT91C_BASE_PIOA->PIO_PER = 0xffffffff;

	// Configure output pins
	AT91C_BASE_PIOA->PIO_CODR = GPIO_OUTPUT_DEFAULT_LOW;
	AT91C_BASE_PIOA->PIO_SODR = GPIO_OUTPUT_DEFAULT_HIGH;
	// Configures as outputs
	AT91C_BASE_PIOA->PIO_OER = GPIO_OUTPUTS;

	// Configure peripheral pins
	AT91C_BASE_PIOA->PIO_ASR = PERIPHERAL_A_FUNCTION;
	AT91C_BASE_PIOA->PIO_BSR = PERIPHERAL_B_FUNCTION;
	// Disable peripheral pins from PIO control
	AT91C_BASE_PIOA->PIO_PDR = PERIPHERAL_A_FUNCTION | PERIPHERAL_B_FUNCTION;
}
