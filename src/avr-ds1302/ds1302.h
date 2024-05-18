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

#ifndef AVR_DS1302_H
#define AVR_DS1302_H

/*
  All the configuration that needs be done is to set up the DDR, PORT, and BIT for each 
  DS1302 interface line
*/


#define DS1302_CE_DDR      DDRC
#define DS1302_CE_PORT     PORTC
#define DS1302_CE_PIN      PC5

#define DS1302_IO_DDR      DDRB
#define DS1302_IO_PORT     PORTB
#define DS1302_IO_PORTIN   PINB
#define DS1302_IO_PIN      PB4

#define DS1302_SCLK_DDR    DDRB
#define DS1302_SCLK_PORT   PORTB
#define DS1302_SCLK_PIN    PB3


void ds1302_init();
void ds1302_transact(uint8_t command, uint8_t bytes, uint8_t* buffer);

#endif


