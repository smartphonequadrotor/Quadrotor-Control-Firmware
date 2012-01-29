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

#ifndef _PINS_H_
#define _PINS_H_

/* AT91SAM7S161.h doesn't define all pins, so we define them here */
#define PA_0  (1 << 0)
#define PA_1  (1 << 1)
#define PA_2  (1 << 2)
#define PA_3  (1 << 3)
#define PA_4  (1 << 4)
#define PA_5  (1 << 5)
#define PA_6  (1 << 6)
#define PA_7  (1 << 7)
#define PA_8  (1 << 8)
#define PA_9  (1 << 9)
#define PA_10 (1 << 10)
#define PA_11 (1 << 11)
#define PA_12 (1 << 12)
#define PA_13 (1 << 13)
#define PA_14 (1 << 14)
#define PA_15 (1 << 15)
#define PA_16 (1 << 16)
#define PA_17 (1 << 17)
#define PA_18 (1 << 18)
#define PA_19 (1 << 19)
#define PA_20 (1 << 20)
#define PA_21 (1 << 21)
#define PA_22 (1 << 22)
#define PA_23 (1 << 23)
#define PA_24 (1 << 24)
#define PA_25 (1 << 25)
#define PA_26 (1 << 26)
#define PA_27 (1 << 27)
#define PA_28 (1 << 28)
#define PA_29 (1 << 29)
#define PA_30 (1 << 30)
#define PA_31 (1 << 31)

/* Define user friendly names for buttons */
#define BUTTON_1 PA_27
#define BUTTON_2 PA_28
#define BUTTON_3 PA_29
#define BUTTON_4 PA_30

/* Define user friendly names for LEDs */
#define LED_1    PA_16
#define LED_2    PA_17
#define LED_3    PA_18
#define LED_4    PA_19

/*
 *  Function that initializes the pins of the QCB.
 */
void pins_init(void);

#endif // _PINS_H_
