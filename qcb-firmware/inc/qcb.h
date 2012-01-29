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

#ifndef _QCB_H_
#define _QCB_H_

/* File for QCB specific defines, variables, and functions that don't belong
 * anywhere else.
 */

/* Defines for peripheral IDs. Used in the PMC Peripheral Clock enable
 * (PMC_PCER), disable (PMC_PCDR), and status (PMC_PCSR).
 * Peripheral IDs 0 and 1 can not have their clocks enabled/disabled.
 * Peripheral ID 4 (ADC) has its clock automatically started.
 * Peripheral IDs 3, 15-29 are reserved.
 */
#define PID_2  (1 << 2)  // PIOA  - Parallel I/O Controller A
#define PID_5  (1 << 5)  // SPI   - Serial Peripheral Interface
#define PID_6  (1 << 6)  // US0   - USART0
#define PID_7  (1 << 7)  // US1   - USART1
#define PID_8  (1 << 8)  // SSC   - Synchronous Serial Controller
#define PID_9  (1 << 9)  // TWI   - Two-wire Interface
#define PID_10 (1 << 10) // PWMC  - PWM Controller
#define PID_11 (1 << 11) // UDP   - USB Device Port
#define PID_12 (1 << 12) // TC0   - Timer/Counter 0
#define PID_13 (1 << 13) // TC1   - Timer/Counter 1
#define PID_14 (1 << 14) // TC2   - Timer/Counter 2
#define PID_30 (1 << 30) // AIC   - Advanced Interrupt Controller
#define PID_31 (1 << 31) // AIC   - Advanced Interrupt Controller

#endif // _QCB_H_
