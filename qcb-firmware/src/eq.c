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

#define EQ_NUM_EVENTS       32
#define EQ_NUM_TIMER_EVENTS 32

typedef struct eq_event_t
{
	eq_handler callback;
	void* buffer;
	uint16_t buffer_size;
} eq_event_t;

typedef struct eq_event_buffer_t
{
	uint8_t read_index;
	uint8_t write_index;
	eq_event_t events[EQ_NUM_EVENTS];
	uint8_t num_events;
} eq_event_buffer_t;

typedef struct eq_timer_event_t
{
	eq_timer_handler callback;
	period_t period;
	timer_type_t type;
} eq_timer_event_t;

typedef struct eq_timer_event_buffer_t
{
	eq_timer_event_t events[EQ_NUM_TIMER_EVENTS];
	uint8_t num_events;
} eq_timer_event_buffer_t;

static eq_event_buffer_t event_buffer;
//static eq_timer_event_buffer_t timer_event_buffer;

void eq_init(void)
{

}

void eq_post(eq_handler callback, void* buffer, uint16_t buffer_size)
{
	uint8_t new_event_index;

	// Check that parameters are acceptable
	if(	(callback != NULL) &&
		((buffer == NULL && buffer_size == 0) || (buffer != NULL && buffer_size != 0)))
	{
		// The fact that we're posting an event means that it is not possible
		// for an event to be dispatched at the same time.
		// The only action that needs to occur with interrupts disabled is an
		// event being reserved in the circular buffer. The actual population
		// of the event can occur with interrupts enabled.
		interrupts_disable();
		if(event_buffer.num_events < EQ_NUM_EVENTS)
		{
			event_buffer.num_events++;
			new_event_index = event_buffer.write_index++;
			if(event_buffer.write_index == EQ_NUM_EVENTS)
			{
				event_buffer.write_index = 0;
			}
		}
		interrupts_enable();

		event_buffer.events[new_event_index].callback = callback;
		event_buffer.events[new_event_index].buffer = buffer;
		event_buffer.events[new_event_index].buffer_size = buffer_size;
	}
}

void eq_post_timer(eq_timer_handler callback, uint32_t period, timer_type_t type)
{

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
		if(event_buffer.read_index == EQ_NUM_EVENTS)
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