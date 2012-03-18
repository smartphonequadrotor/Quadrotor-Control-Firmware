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

#ifndef _EQ_H_
#define _EQ_H_

#include "qcb.h"

typedef void(*eq_handler)(uint8_t[], uint8_t);
typedef void(*eq_timer_handler)(void);

typedef enum
{
	eq_timer_periodic,
	eq_timer_one_shot
} timer_type_t;

typedef uint32_t period_t;

void eq_init(void);
void eq_post(eq_handler callback, void* buffer, uint8_t buffer_size);
void eq_post_timer(eq_timer_handler callback, period_t period, timer_type_t type);
void eq_dispatch(void);
void eq_dispatch_timers(void);

#endif // _EQ_H_
