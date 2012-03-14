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

#include "circular_buffer.h"

void cb_init(circular_buffer_t* circ_buffer, uint8_t* data_buffer, uint16_t data_buffer_size)
{
	circ_buffer->data = data_buffer;
	circ_buffer->capacity = data_buffer_size;
	circ_buffer->size = 0;
	circ_buffer->read_index = 0;
	circ_buffer->write_index = 0;
}

void cb_add_byte(circular_buffer_t* circ_buffer, uint8_t byte)
{
	circ_buffer->data[circ_buffer->write_index++] = byte;
	if(circ_buffer->write_index == circ_buffer->capacity)
	{
		circ_buffer->write_index = 0;
	}
}

void cb_pop_byte(circular_buffer_t* circ_buffer, uint8_t* byte)
{
	*byte = circ_buffer->data[circ_buffer->read_index++];
	if(circ_buffer->read_index == circ_buffer->capacity)
	{
		circ_buffer->read_index = 0;
	}
}

void cb_remove_bytes(circular_buffer_t* circ_buffer, uint16_t num_bytes)
{
	if(circ_buffer->size < num_bytes)
	{
		num_bytes = circ_buffer->size;
	}
	circ_buffer->size -= num_bytes;

	circ_buffer->read_index += num_bytes;
	if(circ_buffer->read_index >= circ_buffer->capacity)
	{
		circ_buffer->read_index = (circ_buffer->read_index - circ_buffer->capacity);
	}
}
