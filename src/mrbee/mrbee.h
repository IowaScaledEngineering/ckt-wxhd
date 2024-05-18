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

#ifndef MRBEE_H
#define MRBEE_H

#include "mrbus.h"


// Packet component defines
#define MRBEE_PKT_DEST  MRBUS_PKT_DEST
#define MRBEE_PKT_SRC   MRBUS_PKT_SRC
#define MRBEE_PKT_LEN   MRBUS_PKT_LEN
#define MRBEE_PKT_CRC_L MRBUS_PKT_CRC_L
#define MRBEE_PKT_CRC_H MRBUS_PKT_CRC_H
#define MRBEE_PKT_TYPE  MRBUS_PKT_TYPE
#define MRBEE_BUFFER_SIZE MRBUS_BUFFER_SIZE

// Status Masks
#define MRBEE_RX_PKT_READY  0x01
#define MRBEE_TX_BUF_ACTIVE 0x40
#define MRBEE_TX_PKT_READY  0x80

// Variables used by MRBee applications
extern volatile uint8_t mrbee_rx_buffer[MRBUS_BUFFER_SIZE];
extern volatile uint8_t mrbee_tx_buffer[MRBUS_BUFFER_SIZE];
extern volatile uint8_t mrbee_state;
extern volatile uint8_t mrbee_rssi;



uint8_t mrbeePacketTransmit(void);
void mrbeeInit(void);
void mrbeePoll(void);

#endif // MRBEE_H
