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

void init(void);

extern void fiq_handler(void);
extern void default_handler(void);
extern void spurious_handler(void);

// Initializes processor
void init(void)
{
	uint8_t  i;

	// Watchdog needs to be disabled
	AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

	// User reset needs to be enabled (2^(4+1) slow clock cycles = 1ms)
	// 0xA5 is the "password"
	AT91C_BASE_RSTC->RSTC_RMR = ((0xA5 << 24) | AT91C_RSTC_URSTEN | (4 << 8));

	// Flash needs to be set to the proper speed
	AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_1FWS; // 2 cycle reads for Freq>30MHz

	// Main oscillator needs to be enabled
	AT91C_BASE_PMC->PMC_MOR = (AT91C_CKGR_OSCOUNT & (0x40 << 8)) | AT91C_CKGR_MOSCEN;
	while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));

	// PLL needs to be started up
	AT91C_BASE_PMC->PMC_PLLR = (
			AT91C_CKGR_USBDIV_1 |
			AT91C_CKGR_OUT_0 |
			(16 << 8) |
			(AT91C_CKGR_MUL & (72 << 16)) |
			(AT91C_CKGR_DIV & 14)
		);
	while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK));

	// Master clock needs to be selected
	// Wait for it to be ready (should be already)
	while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

	// Switch to slow clock
	AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
	while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

	// Now to fast clock
	AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;
	while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

	// Enable protect mode for use with debugger, irq handler must support this
	AT91C_BASE_AIC->AIC_DCR = AT91C_AIC_DCR_PROT;

	// Make sure interrupts are disabled and cleared
	AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;
	AT91C_BASE_AIC->AIC_ICCR = 0xFFFFFFFF;

	// Make sure all 8 levels of interrupt have been ended
	for(i = 0; i < 8; ++i)
	{
		AT91C_BASE_AIC->AIC_EOICR = 0;
	}

	// Make sure Fast Forcing is disabled
	AT91C_BASE_AIC->AIC_FFDR = 0xFFFFFFFE; // Last bit is reserved

	// Default interrupt handlers need to be set
	// Handler for FIQ
	AT91C_BASE_AIC->AIC_SVR[0] = (uint32_t)fiq_handler;

	// Handler for everything else
	for(i = 1; i < 31; ++i)
	{
		AT91C_BASE_AIC->AIC_SVR[i] = (uint32_t)default_handler;
	}

	// Spurious interrupt handler
	AT91C_BASE_AIC->AIC_SPU = (uint32_t)spurious_handler;
}
