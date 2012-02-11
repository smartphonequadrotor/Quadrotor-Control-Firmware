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

#include <stdbool.h>
#include <stdint.h>

#include "AT91SAM7S161.h"

#define PWMC_CMR_CLKA       0x0B
#define PWM_PERIOD          2000
#define PWM_BASE_DUTY       110
#define PWM_OFF_DUTY        90

// Test firmware defines
#define CMD_RECEIVING       0
#define CMD_TRANSMITTING    1

#define MSG_PROMPT          0
#define MSG_ERROR           1
#define MSG_GET             2
#define MSG_PWM             3
#define MSG_SET             4

#define NUM_SEC_BEFORE_AUTO_KILL_ESC   5
#define RTT_1HZ_FREQ        0x00008000

#define NUM_PWM_CH          4

// Forward declarations
static void send_buffer(const char* ps8_bytes, uint16_t length);
static void send_byte(int8_t s8_byte);

int main(void)
{
	uint8_t u8_rx_char = 0;
	uint8_t u8_cmd_state = CMD_TRANSMITTING;
	uint8_t u8_num_bytes = 0;
	uint8_t u8_tx_msg = MSG_PROMPT;
	uint8_t u8_cmd_type = 0;
	uint8_t u8_cmd_num = 0;
	uint8_t u8_cmd_val = 0;

	//------------------------------------------------------------------------
	// Configure GPIO pins
	//------------------------------------------------------------------------

	// Enable clock to Parallel IO Control A (GPIOs)
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_PIOA);

	// PA3, PA4 - I2C bus
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA3 | AT91C_PIO_PA4;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA3 | AT91C_PIO_PA4;

	// PA5 - USART0 Rx
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA5;

	// PA6 - USART0 Tx
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA6;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA6;

	// PA8 - BT_EN (Bluetooth enable)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA8;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA8;

	// PA9 - CS1 (Chip select 1)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA9;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA9;

	// PA10 - EN_1 (Expansion module 1 enable)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA10;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA10;

	// PA11 - CS0 (Chip select 0)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA11;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA11;

	// PA12 - MISO
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA12;

	// PA13 - MOSI
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA13;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA13;

	// PA14 - SCK (SPI clock)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA14;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA14;

	// PA15 - EN_2 (Expansion module 2 enable)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA15;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA15;

	// PA16 - LED0
	// Note: Drive low to turn on
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA16;
	// Off by default
	AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA16;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA16;

	// PA17 - LED1
	// Note: Drive low to turn on
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA17;
	// Off by default
	AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA17;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA17;

	// PA18 - LED2
	// Note: Drive low to turn on
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA18;
	// Off by default
	AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA18;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA18;

	// PA19 - LED3
	// Note: Drive low to turn on
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA19;
	// Off by default
	AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA19;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA19;

	// PA20 - EN_3 (Expansion module 3 enable)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA20;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA20;

	// PA21, PA22 reserved for USART1

	// PA23 - EN_3 (Expansion module 4 enable)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA23;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA23;

	// RTS1 and CTS1 are mixed up on rev 1 of the board
	// PA24 - RTS1 (actually CTS1 which is why its an output)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA24;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA24;
	// Need to set CTS low for BT module to send data
	AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA24;

	// PA25 - CTS1 (actually RTS1 which is why its an input)
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA25;

	// PA26 - NC

	// PA27 - BTN0
	// Note: Line is pulled low when pressed
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA27;

	// PA28 - BTN1
	// Note: Line is pulled low when pressed
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA28;

	// PA29 - BTN2
	// Note: Line is pulled low when pressed
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA29;

	// PA30 - BTN3
	// Note: Line is pulled low when pressed
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA30;

	// PA31 - PWR_CNTL
	// Note: drive low to enable ESCs
	AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA31;
	// Set pin 31 high before enabling output
	// so it doesn't go low and turn on ESCs right away
	AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA31;
	AT91C_BASE_PIOA->PIO_OER = AT91C_PIO_PA31;

	//------------------------------------------------------------------------
	// Configure USART1
	//------------------------------------------------------------------------

	// Enable clock to USART1
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US1);

	// Select the peripheral function for pins 21, 22, 24, 25
	// Peripheral A is the default for these pins after reset
	// Disable PIO control of pins 21, 22, 24, 25 so the peripheral use them
	// READ THIS: PA24 and PA25 are mixed up, peripheral can't use them
	AT91C_BASE_PIOA->PIO_PDR = (AT91C_PIO_PA21 | AT91C_PIO_PA22); // | AT91C_PIO_PA24 | AT91C_PIO_PA25);

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

	//------------------------------------------------------------------------
	// Configure PWM pins (PA0, PA1, PA2, PA7)
	//------------------------------------------------------------------------

	// Enable clock to PWM peripheral
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_PWMC);

	// Configure pins to use peripheral functions
	// PA0, PA1, PA2 use peripheral function A (default)
	AT91C_BASE_PIOA->PIO_PDR = (AT91C_PIO_PA0 | AT91C_PIO_PA1 | AT91C_PIO_PA2 | AT91C_PIO_PA7);

	// Pin PA7 requires peripheral function B to be used
	AT91C_BASE_PIOA->PIO_BSR = AT91C_PIO_PA7;

	// Configure mode register, configures CLKA for 100114 Hz
	// MCK/16/30= 100114 Hz
	// 0x04 << 8 selects MCK/16
	// 0x1E << 0 selects divisor of 30
	AT91C_BASE_PWMC->PWMC_MR = (0x04 << 8) | (0x1E << 0);

	// Configure all channels to use CLKA, left aligned, start low,
	// and update duty cycle when writing PWM_CUPDX
	for(int i = 0; i < NUM_PWM_CH; ++i)
	{
		AT91C_BASE_PWMC->PWMC_CH[i].PWMC_CMR = PWMC_CMR_CLKA | AT91C_PWMC_CPOL;
		AT91C_BASE_PWMC->PWMC_CH[i].PWMC_CPRDR = PWM_PERIOD;
		AT91C_BASE_PWMC->PWMC_CH[i].PWMC_CDTYR = PWM_OFF_DUTY;
	}

	// Last thing to do is enable all 4 pwm channels
	AT91C_BASE_PWMC->PWMC_ENA = (
			AT91C_PWMC_CHID0 |
			AT91C_PWMC_CHID1 |
			AT91C_PWMC_CHID2 |
			AT91C_PWMC_CHID3
	);

	while(1)
	{
		// Turns on bluetooth power when button 4 is pressed
		if((AT91C_BASE_PIOA->PIO_PDSR & AT91C_PIO_PA30) == 0)
		{
			AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA8;
			AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA19;
		}

		// Check if we need to kill the ESCs
		if(AT91C_BASE_RTTC->RTTC_RTVR >= NUM_SEC_BEFORE_AUTO_KILL_ESC)
		{
			AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA31;
			AT91C_BASE_PIOA->PIO_SODR = AT91C_PIO_PA18;
		}

		// Turn on LED 3 when ESC power is enabled
		if((AT91C_BASE_PIOA->PIO_PDSR & AT91C_PIO_PA31) == 0)
		{
			AT91C_BASE_PIOA->PIO_CODR = AT91C_PIO_PA18;
		}

		// Receive data over uart and set pins appropriately
		if(u8_cmd_state == CMD_RECEIVING)
		{
			// If a character is in the buffer
			if(AT91C_BASE_US1->US_CSR & AT91C_US_RXRDY)
			{
				// Get received character
				u8_rx_char = AT91C_BASE_US1->US_RHR;
				u8_num_bytes++;

				// Parse command
				switch(u8_num_bytes)
				{
				case 1:
					if(u8_rx_char == 'g')
					{
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_GET;
					}
					else if(u8_rx_char == 's')
					{
						u8_cmd_type = u8_rx_char;
					}
					else if(u8_rx_char == 'p')
					{
						u8_cmd_type = u8_rx_char;
					}
					else
					{
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_ERROR;
					}
					break;
				case 2:
					// always a number
					if(u8_cmd_type == 's' && u8_rx_char <= '3' && u8_rx_char >= '0')
					{
						u8_cmd_num = (u8_rx_char - 0x30)*10;
					}
					else if(u8_cmd_type == 'p' && u8_rx_char <= '3' && u8_rx_char >= '0')
					{
						u8_cmd_num = (u8_rx_char - 0x30);
					}
					else
					{
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_ERROR;
					}
					break;
				case 3:
					if(u8_cmd_type == 's' && u8_rx_char <= '9' && u8_rx_char >= '0')
					{
						u8_cmd_num += (u8_rx_char - 0x30);
					}
					else if(u8_cmd_type == 'p' && u8_rx_char == '=')
					{
					}
					else
					{
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_ERROR;
					}
					break;
				case 4:
					if(u8_cmd_type == 's' && u8_rx_char == '=')
					{
					}
					else if(u8_cmd_type == 'p' && u8_rx_char <= '9' && u8_rx_char >= '0')
					{
						u8_cmd_val = u8_rx_char - 0x30;
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_PWM;
					}
					else
					{
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_ERROR;
					}
					break;
				case 5:
					if(u8_cmd_type == 's' && (u8_rx_char == '1' || u8_rx_char == '0'))
					{
						u8_cmd_val = u8_rx_char - 0x30;
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_SET;
					}
					else
					{
						u8_num_bytes = 0;
						u8_cmd_state = CMD_TRANSMITTING;
						u8_tx_msg = MSG_ERROR;
					}
					break;
				default:
					u8_num_bytes = 0;
					u8_cmd_state = CMD_TRANSMITTING;
					u8_tx_msg = MSG_ERROR;
					break;
				}
			}
		}
		else if(u8_cmd_state == CMD_TRANSMITTING)
		{
			uint32_t u32_pin_values;
			uint32_t u32_mask1;
			uint32_t u32_mask2;
			uint8_t u8_b1;
			uint8_t u8_b2;

			switch(u8_tx_msg)
			{
			case MSG_PROMPT:
				send_buffer("\r\n>", 3);
				u8_cmd_state = CMD_RECEIVING;
				break;
			case MSG_ERROR:
				send_buffer("\r\nERR", 5);
				u8_tx_msg = MSG_PROMPT;
				break;
			case MSG_GET:
				u32_pin_values = AT91C_BASE_PIOA->PIO_PDSR;
				u32_mask1 = 0xF0000000;
				u32_mask2 = 0x0F000000;
				u8_b1 = 0;
				u8_b2 = 0;

				send_buffer("\r\nPDSR=0x", 9);
				for(int i = 24; i >= 0; i -= 8)
				{
					u8_b1 = ((u32_pin_values & u32_mask1) >> (i+4)) + 0x30;
					u8_b2 = ((u32_pin_values & u32_mask2) >> i) + 0x30;
					u32_mask1 >>= 8;
					u32_mask2 >>= 8;
					if(u8_b1 > '9')
					{
						u8_b1 += 7;
					}
					if(u8_b2 > '9')
					{
						u8_b2 += 7;
					}
					send_byte(u8_b1);
					send_byte(u8_b2);
				}
				u8_tx_msg = MSG_PROMPT;
				break;
			case MSG_PWM:
				if(u8_cmd_num > 3)
				{
					u8_tx_msg = MSG_ERROR;
				}
				else
				{
					// A valid PWM message resets the ESC kill timer
					AT91C_BASE_RTTC->RTTC_RTMR = AT91C_RTTC_RTTRST | RTT_1HZ_FREQ;

					if(u8_cmd_val == 0)
					{
						AT91C_BASE_PWMC->PWMC_CH[u8_cmd_num].PWMC_CUPDR
								= PWM_OFF_DUTY;
					}
					else
					{
						AT91C_BASE_PWMC->PWMC_CH[u8_cmd_num].PWMC_CUPDR
								= PWM_BASE_DUTY + u8_cmd_val*10;
					}
					send_buffer("\r\nOK!", 5);
					u8_tx_msg = MSG_PROMPT;
				}
				break;
			case MSG_SET:
				send_buffer("\r\nOK!", 5);
				if(u8_cmd_val == 0)
				{
					AT91C_BASE_PIOA->PIO_CODR = (1 << u8_cmd_num);
				}
				else
				{
					AT91C_BASE_PIOA->PIO_SODR = (1 << u8_cmd_num);
				}
				u8_tx_msg = MSG_PROMPT;
				break;
			default:
				u8_tx_msg = MSG_ERROR;
				break;
			}
		}
		else
		{
			u8_cmd_state = CMD_TRANSMITTING;
			u8_tx_msg = MSG_ERROR;
		}

	}

	return 0;
}

static void send_buffer(const char* ps8_bytes, uint16_t length)
{
	uint16_t index = 0;
	while(length > 0)
	{
		send_byte(ps8_bytes[index]);
		index++;
		length--;
	}
}

static void send_byte(int8_t s8_byte)
{
	// Make sure nothing is being transmitted
	while((AT91C_BASE_US1->US_CSR & AT91C_US_TXEMPTY) == 0);

	// Write byte to transmit holding register
	AT91C_BASE_US1->US_THR = s8_byte;

	// Wait for byte to be sent
	while((AT91C_BASE_US1->US_CSR & AT91C_US_TXEMPTY) == 0);
}
