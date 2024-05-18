/*************************************************************************
Title:    MRBee Functions
Authors:  Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License

LICENSE:
    Copyright (C) 2011 Michael Petersen

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
#include <util/delay.h>

#include "mrbee.h"
#include "xbee.h"


/* Variables used by MRBee code */
extern volatile uint8_t xbee_state;

/* Variables used by MRBee applications */
volatile uint8_t mrbee_rx_buffer[MRBUS_BUFFER_SIZE];
volatile uint8_t mrbee_tx_buffer[MRBUS_BUFFER_SIZE];
volatile uint8_t mrbee_state;
volatile uint8_t mrbee_rssi;

uint8_t mrbeePacketTransmit(void)
{
	uint8_t i;
	uint16_t crc16_value = 0x0000;

	//  Return if bus already active.
	//  However, pending packet may already be trashed by application re-writing the buffer
	if (mrbee_state & MRBEE_TX_BUF_ACTIVE) return(1);

	// First Calculate CRC16
	for (i = 0; i < mrbee_tx_buffer[MRBEE_PKT_LEN]; i++)
	{
		if ((i != MRBEE_PKT_CRC_H) && (i != MRBEE_PKT_CRC_L))
		{
			crc16_value = mrbusCRC16Update(crc16_value, mrbee_tx_buffer[i]);
		}
	}
	mrbee_tx_buffer[MRBEE_PKT_CRC_L] = (crc16_value & 0xFF);
	mrbee_tx_buffer[MRBEE_PKT_CRC_H] = ((crc16_value >> 8) & 0xFF);

	mrbee_state |= MRBEE_TX_BUF_ACTIVE;
	
	xbeeAddr16Transmit( mrbee_tx_buffer, mrbee_tx_buffer[MRBEE_PKT_LEN], 0xFFFF, 0 );

	return(0);
}


void mrbeeInit(void)
{
	uint16_t ubrr;
	#undef BAUD
	#define BAUD MRBEE_BAUD
	#include <util/setbaud.h>
	ubrr = UBRR_VALUE;
	#if USE_2X
	ubrr |= 0x8000;
	#endif
	xbeeInit(ubrr);
	mrbee_state = 0;
}


void mrbeePoll(void)
{
	uint8_t i;
	uint8_t len;
	volatile uint8_t *ptr;
	
	/* Get new data and update status flag */
	if( (xbee_state & XBEE_RX_PKT_READY) && !(mrbee_state & MRBEE_RX_PKT_READY) )
	{
		i = xbeeReceivedIdentifier();
		if( (i == 0x80) || (i == 0x81) )
		{
			// RX Packet
			len = xbeeReceivedDataLength();
			if(len > MRBUS_BUFFER_SIZE) len = MRBUS_BUFFER_SIZE;
			ptr = xbeeReceivedData();
			for(i=0; i<len; i++)
			{
				mrbee_rx_buffer[i] = *(ptr+i);
			}
			mrbee_state |= MRBEE_RX_PKT_READY;
			mrbee_rssi = xbeeReceivedRSSI();
		}
		xbeePop();
	}

	if( !(xbee_state & XBEE_TX_BUF_ACTIVE) )
	{
		mrbee_state &= ~MRBEE_TX_BUF_ACTIVE;
	}
}



