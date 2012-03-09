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

#include "qcb.h"
#include "system.h"
#include "pins.h"
#include "us1.h"
#include "twi.h"
#include "eq.h"
#include "qcfp.h"
#include "interrupts.h"

int main(void)
{
	uint8_t twi_byte;

	// Enable peripherals
	system_init();
	pins_init();
	us1_init();
	twi_init();

	// Initialize state machines
	qcfp_init();
	eq_init();

	// Once everything is initialized, enable interrupts globally
	interrupts_enable();

	while(1)
	{
//		eq_dispatch();
//		twi_start_single_byte_write(TWI_GYRO_ADDR);//TWI_ACCEL_ADDR);
//		twi_write_byte(0x27);//0);
//		twi_start_single_byte_read(TWI_GYRO_ADDR);//TWI_ACCEL_ADDR);
//		twi_byte = 0x27;
//		AT91F_TWI_WriteByte(AT91C_BASE_TWI, TWI_GYRO_ADDR, 0, &twi_byte, 1);
//		AT91F_TWI_ReadByte(AT91C_BASE_TWI, TWI_GYRO_ADDR, 0, &twi_byte, 1);
	}
	return 0;
}
