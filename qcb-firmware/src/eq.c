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

#include "eq.h"
#include "interrupts.h"
#include "system.h"
#include <string.h>

#define EQ_MAX_EVENTS        32
#define EQ_MAX_TIMER_EVENTS  32
#define EQ_EVENT_BUFFER_SIZE 32

typedef struct eq_event_t
{
	eq_handler callback;
	uint8_t buffer[EQ_EVENT_BUFFER_SIZE];
	uint8_t buffer_size;
} eq_event_t;

typedef struct eq_event_buffer_t
{
	uint8_t read_index;
	uint8_t write_index;
	eq_event_t events[EQ_MAX_EVENTS];
	uint8_t num_events;
} eq_event_buffer_t;

typedef struct eq_timer_event_t
{
	eq_timer_handler callback;
	period_t period;
	uint32_t last_execution;
	timer_type_t type;
} eq_timer_event_t;

typedef struct eq_timer_event_buffer_t
{
	eq_timer_event_t events[EQ_MAX_TIMER_EVENTS];
	bool event_used[EQ_MAX_TIMER_EVENTS];
	uint8_t num_events;
} eq_timer_event_buffer_t;

static eq_event_buffer_t event_buffer;
static eq_timer_event_buffer_t timer_event_buffer;
static uint32_t eq_timer_last_time;

void eq_init(void)
{
	eq_timer_last_time = system_uptime();
}

void eq_post(eq_handler callback, void* buffer, uint8_t buffer_size)
{
	uint8_t new_event_index;

	// Check that parameters are acceptable
	if(	(callback != NULL) &&
		((buffer == NULL && buffer_size == 0) || (buffer != NULL && buffer_size != 0)) &&
		(buffer_size <= EQ_EVENT_BUFFER_SIZE))
	{
		// The only action that needs to occur with interrupts disabled is an
		// event being reserved in the circular buffer. The actual population
		// of the event can occur with interrupts enabled.
		interrupts_disable();
		if(event_buffer.num_events < EQ_MAX_EVENTS)
		{
			event_buffer.num_events++;
			new_event_index = event_buffer.write_index++;
			if(event_buffer.write_index == EQ_MAX_EVENTS)
			{
				event_buffer.write_index = 0;
			}
		}
		interrupts_enable();

		event_buffer.events[new_event_index].callback = callback;
		event_buffer.events[new_event_index].buffer_size = buffer_size;
		if(buffer_size > 0)
		{
			memcpy(event_buffer.events[new_event_index].buffer, buffer, buffer_size);
		}
	}
}

/*
 * Timer events can be posted from any run level and will always execute
 * at the main loop level.
 */
void eq_post_timer(eq_timer_handler callback, uint32_t period, timer_type_t type)
{
	int i;
	uint8_t timer_index;

	interrupts_disable();
	if(timer_event_buffer.num_events >= EQ_MAX_TIMER_EVENTS)
	{
		interrupts_enable();
		return;
	}

	for(i = 0; i < EQ_MAX_TIMER_EVENTS; i++)
	{
		if(timer_event_buffer.event_used[i] == false)
		{
			timer_index = i;
			break;
		}
	}

	if(i == EQ_MAX_TIMER_EVENTS)
	{
		interrupts_enable();
		return;
	}

	timer_event_buffer.events[timer_index].callback = callback;
	timer_event_buffer.events[timer_index].period = period;
	timer_event_buffer.events[timer_index].last_execution = system_uptime();
	timer_event_buffer.events[timer_index].type = type;
	timer_event_buffer.event_used[timer_index] = true;
	timer_event_buffer.num_events++;
	interrupts_enable();
}

void eq_dispatch(void)
{
	eq_event_t execute_me;

	// This is the only function modifying the event queue's read index
	// so it is safe to read and modify without disabling interrupts
	// Reading the number of events without safety is safe since this function
	// is the only one to modify the variable. At worst we get a stale value
	// because an event is posted after the read completes
	if(event_buffer.num_events > 0)
	{
		execute_me = event_buffer.events[event_buffer.read_index];
		event_buffer.read_index++;
		if(event_buffer.read_index == EQ_MAX_EVENTS)
		{
			event_buffer.read_index = 0;
		}

		// Posting an event from an interrupt could modify num_events at the same
		// time so we require safety
		interrupts_disable();
		event_buffer.num_events--;
		interrupts_enable();

		// Execute the event
		execute_me.callback(execute_me.buffer, execute_me.buffer_size);
	}
}

void eq_dispatch_timers(void)
{
	int i;
	uint32_t now = system_uptime();

	if(eq_timer_last_time < now)
	{
		for(i = 0; i < EQ_MAX_TIMER_EVENTS; i++)
		{
			if(timer_event_buffer.event_used[i])
			{
				if((timer_event_buffer.events[i].last_execution + timer_event_buffer.events[i].period) <= now)
				{
					timer_event_buffer.events[i].callback();
					if(timer_event_buffer.events[i].type == eq_timer_one_shot)
					{
						timer_event_buffer.event_used[i] = false;
					}
					else
					{
						timer_event_buffer.events[i].last_execution = now;
					}
				}
			}
		}
	}
}
