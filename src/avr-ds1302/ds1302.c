/*************************************************************************
Title:    AVR Library for DS1302 Real Time Clocks
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan D. Holmes

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
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "ds1302.h"

void ds1302_init()
{
	// Drop the CE line low - don't want to accidentally do anything
	DS1302_CE_PORT &= ~_BV(DS1302_CE_PIN);
	DS1302_CE_DDR |= _BV(DS1302_CE_PIN);

	DS1302_SCLK_PORT &= ~_BV(DS1302_SCLK_PIN);
	DS1302_SCLK_DDR |= _BV(DS1302_SCLK_PIN);

	DS1302_IO_PORT &= ~_BV(DS1302_IO_PIN);
	DS1302_IO_DDR |= _BV(DS1302_IO_PIN);
	
	DDRD |= _BV(PD3);
	PORTD |= _BV(PD3);
}



void ds1302_clock_cycle()
{
	DS1302_SCLK_PORT |= _BV(DS1302_SCLK_PIN);
	_delay_us(1);
	DS1302_SCLK_PORT &= ~_BV(DS1302_SCLK_PIN);
}


void ds1302_transact(uint8_t command, uint8_t bytes, uint8_t* buffer) 
{
	uint8_t bit, byte;
	uint8_t read = command & 0x01;
	
	DS1302_CE_PORT &= ~_BV(DS1302_CE_PIN);
	DS1302_SCLK_PORT &= ~_BV(DS1302_SCLK_PIN);

	// I/O pin to output
	DS1302_IO_DDR |= _BV(DS1302_IO_PIN);
	_delay_us(4); // Satisfy tcwh timing - can be as low as 1uS at 5V

	DS1302_CE_PORT |= _BV(DS1302_CE_PIN);
	_delay_us(4); // Satisfy tcc timing - can be as low as 1uS at 5V		


	byte = command;
	// Send command, LSB first
	// send the command 
	for (bit=0; bit<8; bit++) 
	{
		if (byte & 0x01)
			DS1302_IO_PORT |= _BV(DS1302_IO_PIN);
		else
			DS1302_IO_PORT &= ~_BV(DS1302_IO_PIN);
		_delay_us(1);
		ds1302_clock_cycle();
		byte >>= 1;
	}

	if (read)
	{
		memset(buffer, 0, bytes);
		DS1302_IO_DDR &= ~_BV(DS1302_IO_PIN);
		for(byte=0; byte<bytes; byte++)
		{
			for (bit=0; bit<8; bit++) 
			{
				if (DS1302_IO_PORTIN & _BV(DS1302_IO_PIN))
					buffer[byte] |= _BV(bit);
				ds1302_clock_cycle();
				_delay_us(1);
			}
		}
	} else {
		for(byte=0; byte<bytes; byte++)
		{
			for (bit=0; bit<8; bit++) 
			{
				if (buffer[byte] & _BV(bit))
					DS1302_IO_PORT |= _BV(DS1302_IO_PIN);
				else
					DS1302_IO_PORT &= ~_BV(DS1302_IO_PIN);
				_delay_us(1);
				ds1302_clock_cycle();
			}
		}
	}

	DS1302_SCLK_PORT &= ~_BV(DS1302_SCLK_PIN);
	DS1302_CE_PORT &= ~_BV(DS1302_CE_PIN);

	// I/O pin to output
	DS1302_IO_DDR |= _BV(DS1302_IO_PIN);
	return;
}



