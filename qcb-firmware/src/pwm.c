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

#include "pwm.h"

#define PWM_NUM_MOTORS      4
#define PWMC_CMR_CLKA       0x0B
#define PWM_PERIOD          2000
#define PWM_OFF_DUTY        90
#define PWM_BASE_DUTY       116
#define PWM_MAX_DUTY        190

void pwm_init(void)
{
	// Enable clock to PWM peripheral
	AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_PWMC);

	// Configure mode register, configures CLKA for 100114 Hz
	// MCK/16/30= 100114 Hz
	// 0x04 << 8 selects MCK/16
	// 0x1E << 0 selects divisor of 30
	AT91C_BASE_PWMC->PWMC_MR = (0x04 << 8) | (0x1E << 0);

	// Configure all channels to use CLKA, left aligned, start low,
	// and update duty cycle when writing PWM_CUPDX
	for(int i = 0; i < PWM_NUM_MOTORS; ++i)
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
}

void pwm_off(pwm_motor m)
{
	AT91C_BASE_PWMC->PWMC_CH[m].PWMC_CUPDR = PWM_OFF_DUTY;
}

void pwm_off_all(void)
{
	pwm_off(pwm_motor1);
	pwm_off(pwm_motor2);
	pwm_off(pwm_motor3);
	pwm_off(pwm_motor4);
}

void pwm_set(pwm_motor m, int32_t value)
{
	if(value > (PWM_MAX_DUTY - PWM_BASE_DUTY))
	{
		value = (PWM_MAX_DUTY - PWM_BASE_DUTY);
	}

	if(value < 0)
	{
		value = 0;
	}

	value += PWM_BASE_DUTY;
	while(AT91C_BASE_PWMC->PWMC_CH[m].PWMC_CCNTR == (AT91C_BASE_PWMC->PWMC_CH[m].PWMC_CPRDR - 1) || AT91C_BASE_PWMC->PWMC_CH[m].PWMC_CCNTR == 0 || AT91C_BASE_PWMC->PWMC_CH[m].PWMC_CCNTR == 1 )
	{}
	AT91C_BASE_PWMC->PWMC_CH[m].PWMC_CUPDR = value;
}

void pwm_set_all(uint8_t value)
{
	pwm_set(pwm_motor1, value);
	pwm_set(pwm_motor2, value);
	pwm_set(pwm_motor3, value);
	pwm_set(pwm_motor4, value);
}
