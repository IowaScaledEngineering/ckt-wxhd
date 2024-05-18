/************************************************************************
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

************************************************************************/

#ifndef XBEE_H
#define XBEE_H

/*
** constants and macros
*/

#ifndef XBEE_UART_TX_BUFFER_SIZE
#define XBEE_UART_TX_BUFFER_SIZE  64
#endif

#ifndef XBEE_UART_RX_BUFFER_SIZE
#define XBEE_UART_RX_BUFFER_SIZE  64
#endif

#ifndef XBEE_UART_RX_BUFFER_DEPTH
#define XBEE_UART_RX_BUFFER_DEPTH 4
#endif

/* Status flags */
#define XBEE_RX_PKT_READY  0x01
#define XBEE_RX_BUF_FULL   0x02
#define XBEE_RX_ESCAPED    0x04
#define XBEE_TX_BUF_ACTIVE 0x40

/* Define the UART port and registers used for XBee communication */
/* Follows the format of the AVR UART library by Fleury/Sharpe    */
#if defined(__AVR_ATmega48__) ||defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega48P__) ||defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
      defined(__AVR_ATmega328__) ||defined(__AVR_ATmega328P__)
#define XBEE_ATMEGA_USART0
#define XBEE_UART_RX_INTERRUPT  USART_RX_vect
#define XBEE_UART_TX_INTERRUPT  USART_UDRE_vect
#define XBEE_UART_STATUS        UCSR0A
#define XBEE_UART_CONTROL       UCSR0B
#define XBEE_UART_DATA          UDR0
#define XBEE_UART_UDRIE         UDRIE0
#define XBEE_PORT               PORTD
#define XBEE_PIN                PIND
#define XBEE_DDR                DDRD
#ifndef XBEE_CTS
#define XBEE_CTS                3       /* PD3 */
#endif
#ifndef XBEE_RTS
#define XBEE_RTS                2       /* PD2 */
#endif
#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
/* ATmega with two USART */
#define XBEE_ATMEGA_USART1
#define XBEE_UART_RX_INTERRUPT  USART1_RX_vect
#define XBEE_UART_TX_INTERRUPT  USART1_UDRE_vect
#define XBEE_UART_STATUS        UCSR1A
#define XBEE_UART_CONTROL       UCSR1B
#define XBEE_UART_DATA          UDR1
#define XBEE_UART_UDRIE         UDRIE1
#define XBEE_PORT               PORTD
#define XBEE_PIN                PIND
#define XBEE_DDR                DDRD
#ifndef XBEE_CTS
#define XBEE_CTS                5       /* PD5 */
#endif
#ifndef XBEE_RTS
#define XBEE_RTS                6       /* PD6 */
#endif
#else
#error "no UART definition for MCU available"
#endif


/*
** function prototypes
*/

uint8_t xbeeAddr16Transmit(volatile uint8_t *s, uint8_t len, uint16_t addr, uint8_t frame);
uint8_t xbeeEscapeChar(uint8_t c);

uint8_t xbeeReceivedPacketLength(void);
uint8_t xbeeReceivedDataLength(void);
uint16_t xbeeReceivedAddr16(void);
uint8_t xbeeReceivedIdentifier(void);
uint8_t xbeeReceivedRSSI(void);
volatile uint8_t *xbeeReceivedPacket(void);
volatile uint8_t *xbeeReceivedData(void);
void xbeePop(void);

uint8_t xbeeDepthIn(void);
uint8_t xbeeDepthOut(void);
uint8_t xbeeChecksum(void);

void xbeeInit(uint16_t ubrr);

#endif // XBEE_H 

