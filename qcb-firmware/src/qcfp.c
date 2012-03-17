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

#include "qcfp.h"
#include "eq.h"

#define COBS_TERM_BYTE (0)

static void qcfp_handle_packet(uint8_t packet[], uint8_t length);

void qcfp_init(void)
{

}

void qcfp_data_received(uint8_t buffer[], uint8_t buffer_size)
{
	typedef enum cobs_state
	{
		cobs_decode,
		cobs_copy,
		cobs_sync,
	} cobs_state;

	static cobs_state decode_state = cobs_sync;
	static uint8_t incoming_packet[QCFP_MAX_PACKET_SIZE+2]; // Extra space in case of overflow
	static uint8_t packet_size = 0; // Counts packet size
	static uint8_t byte_count = 0; // Counts number of encoded bytes

	int i;

	// Decode data from buffer
	for(i = 0; i < buffer_size; i++)
	{
		if(packet_size > QCFP_MAX_PACKET_SIZE)
		{
			decode_state = cobs_sync;
		}

		switch(decode_state)
		{
		case cobs_decode:
			if(buffer[i] == COBS_TERM_BYTE)
			{
				if(packet_size > 0)
				{
					eq_post(qcfp_handle_packet, incoming_packet, packet_size);
				}
				packet_size = 0;
				byte_count = 0;
			}
			else
			{
				byte_count = buffer[i];
				decode_state = cobs_copy;
			}
			break;
		case cobs_copy:
			if(buffer[i] == COBS_TERM_BYTE)
			{
				// Got a zero when expecting data, re-sync
				byte_count = 0;
				packet_size = 0;
				decode_state = cobs_decode;
			}
			else
			{
				if(byte_count > 1)
				{
					incoming_packet[packet_size++] = buffer[i];
					byte_count--;
				}
				else // byte_count == 1
				{
					incoming_packet[packet_size++] = buffer[i];
					byte_count--;
					// We add two bytes to the packet here which could result in the packet
					// being larger than the max size which is why the packet has a couple of
					// extra bytes in it
					incoming_packet[packet_size++] = 0;
					decode_state = cobs_decode;
				}
			}
			break;
		case cobs_sync:
		default:
			packet_size = 0;
			byte_count = 0;
			if(buffer[i] == COBS_TERM_BYTE)
			{
				decode_state = cobs_decode;
			}
			break;
		}
	}
}

static void qcfp_handle_packet(uint8_t packet[], uint8_t length)
{
	int i = 0;
	i++;
}
