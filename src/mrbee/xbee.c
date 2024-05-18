/*************************************************************************
Title:    XBee API Interface Functions
Author:   Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License

LICENSE:
    Copyright (C) 2011 Michael Petersen

    UART code derived from AVR UART library by Peter Fleury, and as
    modified by Tim Sharpe.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "xbee.h"


/* Variables used to write to XBee module */
static volatile uint8_t xbee_tx_buffer[XBEE_UART_TX_BUFFER_SIZE];
static volatile uint8_t xbee_tx_offset;
static volatile uint8_t xbee_tx_end;
static volatile uint8_t xbee_tx_checksum;

/* Variables used to read from XBee module */
static volatile uint8_t xbee_rx_buffer[XBEE_UART_RX_BUFFER_SIZE * XBEE_UART_RX_BUFFER_DEPTH];
static volatile uint8_t xbee_rx_index;     // Character index for incoming data
static volatile uint8_t xbee_rx_depth_in;  // Depth index for incoming data
static volatile uint8_t xbee_rx_depth_out; // Depth index for outgoing data
static volatile uint8_t xbee_rx_checksum;

/* Status variables used by XBee applications */
volatile uint8_t xbee_state;


uint8_t xbeeAddr16Transmit(volatile uint8_t *s, uint8_t len, uint16_t addr, uint8_t frame)
{
	uint8_t i;

	//  Wait until previous transmission is complete
	loop_until_bit_is_clear(XBEE_UART_CONTROL, XBEE_UART_UDRIE);
	
	xbee_tx_end = 0;
	xbee_tx_checksum = 0;

	// API Frame Delimeter
	xbee_tx_buffer[xbee_tx_end++] = 0x7E;

	// Length
	xbee_tx_buffer[xbee_tx_end++] = 0x00;
	xbeeEscapeChar(5 + len);

	// *** Begin Checksum
	// Transmit, 16-bit address
	xbee_tx_checksum += xbeeEscapeChar(0x01);

	// Frame ID
	xbee_tx_checksum += xbeeEscapeChar(frame);

	// Address
	xbee_tx_checksum += xbeeEscapeChar(addr >> 8);
	xbee_tx_checksum += xbeeEscapeChar(addr & 0xFF);

	// Options
	xbee_tx_checksum += xbeeEscapeChar(0x00);

	// Data to transmit
	for (i = 0; i < len; i++)
	{
		xbee_tx_checksum += xbeeEscapeChar(s[i]);
	}

	xbee_tx_checksum = 0xFF - xbee_tx_checksum;
	xbeeEscapeChar(xbee_tx_checksum);

	// Set interrupt to start transmitting
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		xbee_state |= XBEE_TX_BUF_ACTIVE;
		XBEE_UART_CONTROL |= _BV(XBEE_UART_UDRIE);
	}
	
	return frame;
}


uint8_t xbeeEscapeChar(uint8_t c)
{
	if ( (c == 0x7E) || (c == 0x7D) || (c == 0x11) || (c == 0x13) )
	{
		xbee_tx_buffer[xbee_tx_end++] = 0x7D;
		xbee_tx_buffer[xbee_tx_end++] = c ^ 0x20;
	}
	else
	{
		xbee_tx_buffer[xbee_tx_end++] = c;
	}
	return c;
}




uint8_t xbeeReceivedPacketLength(void)
{
	// +2 for length bytes, +1 for checksum byte (not counted in packet length byte)
	return xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 1] + 3;
}

uint8_t xbeeReceivedDataLength(void)
{
	// -1 identifier, -2 source address, -1 RSSI, -1 options
	// Checksum already not counted in packet length byte
	if(xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 2] == 0x80)
	{
		// 64-bit address RX packet
		return (xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 1] - 11);
	}
	else if(xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 2] == 0x81)
	{
		// 16-bit address RX packet
		return (xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 1] - 5);
	}
	else
	{
		return 0;
	}
}

uint16_t xbeeReceivedAddr16(void)
{
	// Returns 16-bit address of received data
	return (xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 3] << 8) + xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 4];
}

uint8_t xbeeReceivedIdentifier(void)
{
	// Returns API Identifier
	return xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 2];
}

uint8_t xbeeReceivedRSSI(void)
{
	// Returns RSSI value
	return xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 5];
}

volatile uint8_t *xbeeReceivedPacket(void)
{
	// Returns pointer to current packet
	return &xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 0];
}

volatile uint8_t *xbeeReceivedData(void)
{
	// Returns pointer to current data
	// +2 length, +1 identifier, +2 source address, +1 RSSI, +1 Options
	return &xbee_rx_buffer[(xbee_rx_depth_out * XBEE_UART_RX_BUFFER_SIZE) + 0] + 7;
}



void xbeePop(void)
{
	if( (xbee_state & XBEE_RX_BUF_FULL) || (xbee_rx_depth_out != xbee_rx_depth_in) )
	{
		//  Buffer is full or pointers are not the same, increment
		xbee_rx_depth_out++;

		//  Wrap around circular buffer
		if(xbee_rx_depth_out >= XBEE_UART_RX_BUFFER_DEPTH)
		{
			xbee_rx_depth_out = 0;
		}
	}

	if(xbee_rx_depth_out == xbee_rx_depth_in)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			xbee_state &= ~XBEE_RX_PKT_READY;
		}
	}

	//  Clear FULL status since we just removed a packet
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		xbee_state &= ~XBEE_RX_BUF_FULL;
	}
}


uint8_t xbeeDepthIn(void)
{
	return xbee_rx_depth_in;
}

uint8_t xbeeDepthOut(void)
{
	return xbee_rx_depth_out;
}

uint8_t xbeeChecksum(void)
{
	return xbee_rx_checksum;
}





void xbeeInit(uint16_t ubrr)
{
	xbee_state = 0;
	
	xbee_tx_offset = 0;  // Initialize transmit pointer
	xbee_tx_end = 0;

	xbee_rx_index = 0;
	xbee_rx_depth_in = 0;
	xbee_rx_depth_out = 0;

#if defined( XBEE_AT90_UART )
	/* set baud rate */
	UBRR = ubrr;

	/* enable UART receiver and transmmitter and receive complete interrupt */
	XBEE_UART_CONTROL = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN);

#elif defined( XBEE_ATMEGA_USART )
	/* Set baud rate */
	if ( ubrr & 0x8000 )
	{
		XBEE_UART_STATUS = (1 << U2X); //Enable 2x speed
		ubrr &= ~0x8000;
	}
	UBRRH = (uint8_t)(ubrr >> 8);
	UBRRL = (uint8_t) ubrr;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	XBEE_UART_CONTROL = _BV(RXCIE) | (1 << RXEN) | (1 << TXEN);

	/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
#else
	UCSRC = (3 << UCSZ0);
#endif

#elif defined ( XBEE_ATMEGA_USART0 )
	/* Set baud rate */
	if ( ubrr & 0x8000 )
	{
		XBEE_UART_STATUS = (1 << U2X0); //Enable 2x speed
		ubrr &= ~0x8000;
	}
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t) ubrr;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	XBEE_UART_CONTROL = _BV(RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

	/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL0
	UCSR0C = (1 << URSEL0) | (3 << UCSZ00);
#else
	UCSR0C = (3 << UCSZ00);
#endif

#elif defined ( XBEE_ATMEGA_UART )
	/* set baud rate */
	if ( ubrr & 0x8000 )
	{
		XBEE_UART_STATUS = (1 << U2X); //Enable 2x speed
		ubrr &= ~0x8000;
	}
	UBRRHI = (uint8_t)(ubrr >> 8);
	UBRR   = (uint8_t) ubrr;

	/* Enable UART receiver and transmitter and receive complete interrupt */
	XBEE_UART_CONTROL = _BV(RXCIE) | (1 << RXEN) | (1 << TXEN);

#elif defined ( XBEE_ATMEGA_USART1 )
	/* Set baud rate */
	if ( ubrr & 0x8000 )
	{
		XBEE_UART_STATUS = (1 << U2X1); //Enable 2x speed
		ubrr &= ~0x8000;
	}
	UBRR1H = (uint8_t)(ubrr >> 8);
	UBRR1L = (uint8_t) ubrr;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	XBEE_UART_CONTROL = _BV(RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

	/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL1
	UCSR1C = (1 << URSEL1) | (3 << UCSZ10);
#else
	UCSR1C = (3 << UCSZ10);
#endif

#endif

#ifndef XBEE_IGNORE_FLOW
	// Wait for XBee to start (assert /CTS line low)
	do {wdt_reset();} while (bit_is_set(XBEE_PIN, XBEE_CTS));  // Watchdog aware loop
//	loop_until_bit_is_clear(XBEE_PIN, XBEE_CTS);
#endif

	//  Disable UART interrupt until needed
	XBEE_UART_CONTROL &= ~_BV(XBEE_UART_UDRIE);
}





ISR(XBEE_UART_TX_INTERRUPT)
{
#ifndef XBEE_IGNORE_FLOW
	do {wdt_reset();} while (bit_is_set(XBEE_PIN, XBEE_CTS));  // Watchdog aware loop
//	loop_until_bit_is_clear(XBEE_PIN, XBEE_CTS);
#endif
	XBEE_UART_DATA = xbee_tx_buffer[xbee_tx_offset];  //  Get next byte and write to UART
	xbee_tx_offset++;
	if ( (xbee_tx_offset >= xbee_tx_end) || (xbee_tx_offset == XBEE_UART_TX_BUFFER_SIZE) )
	{
		//  Done sending data to UART, disable UART interrupt
		XBEE_UART_CONTROL &= ~_BV(XBEE_UART_UDRIE);
		xbee_state &= ~XBEE_TX_BUF_ACTIVE;
		xbee_tx_offset = 0;
	}
}

ISR(XBEE_UART_RX_INTERRUPT)
{
	uint8_t data;
	
	data = XBEE_UART_DATA;

	if (!(xbee_state & XBEE_RX_BUF_FULL))
	{
		if (data == 0x7E)
		{
			// Frame start
			xbee_rx_index = 0;
			xbee_rx_checksum = 0;
		}
		else if (data == 0x7D)
			{
			// Next character is escaped
			xbee_state |= XBEE_RX_ESCAPED;
		}
		else
		{
			if (xbee_state & XBEE_RX_ESCAPED)
			{
				// Unescape character
				data = data ^ 0x20;

				// Clear escape flag
				xbee_state &= ~XBEE_RX_ESCAPED;
			}
			xbee_rx_buffer[(xbee_rx_depth_in * XBEE_UART_RX_BUFFER_SIZE) + xbee_rx_index] = data;

			if (xbee_rx_index > 1)
			{
				// After length bytes
				if (xbee_rx_index == (xbee_rx_buffer[(xbee_rx_depth_in * XBEE_UART_RX_BUFFER_SIZE) + 1] + 2))
				{
					// Checksum byte
					xbee_rx_checksum += data;  // Add one last time
					if (xbee_rx_checksum == 0xFF)
					{
						// Validate checksum.  If correct, advance depth pointer
						xbee_rx_depth_in++;
						if (xbee_rx_depth_in >= XBEE_UART_RX_BUFFER_DEPTH)
						{
							xbee_rx_depth_in = 0;
						}

						if (xbee_rx_depth_in == xbee_rx_depth_out)
						{
							xbee_state |= XBEE_RX_BUF_FULL;
							//  FULL status cleared by Pop function
						}
						xbee_state |= XBEE_RX_PKT_READY;
					}
					else
					{
						// Checksum wrong, trash packet (reset index to zero)
						xbee_rx_index = 0;
					}
				}
				else
				{
					xbee_rx_checksum += data;
				}
			}
			xbee_rx_index++;
		}
	}
}



