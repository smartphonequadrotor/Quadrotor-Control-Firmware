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

#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#include "qcb.h"

typedef struct circular_buffer_t
{
	uint16_t read_index;
	uint16_t write_index;
	uint16_t capacity;
	uint16_t size;
	uint8_t* data;
} circular_buffer_t;

/*
 * Initializes circ_buffer with the buffer data_buffer with the size
 * data_buffer_size.
 */
void cb_init(circular_buffer_t* circ_buffer, uint8_t* data_buffer, uint16_t data_buffer_size);

/*
 * Adds the byte byte to the buffer circ_buffer.
 * ***NOTE***
 * This function only adds a byte, it does not update any of the fields in the
 * circular_buffer_t structure. This is left to the caller so that they can
 * lock the specific critical regions necessary.
 */
void cb_add_byte(circular_buffer_t* circ_buffer, uint8_t byte);

/*
 * Removes a byte from the buffer circ_buffer and places it in *byte.
 * ***NOTE***
 * This function only removes a byte, it does not update any of the fields in
 * the circular_buffer_t structure. This is left to the caller so that they can
 * lock the specific critical regions necessary.
 */
void cb_pop_byte(circular_buffer_t* circ_buffer, uint8_t* byte);

void cb_remove_bytes(circular_buffer_t* circ_buffer, uint16_t num_bytes);

#endif // _CIRCULAR_BUFFER_H_
