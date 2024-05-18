/*************************************************************************
Title:    MRBus Weather Head Unit
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan D. Holmes & Michael Petersen

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
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "ds1302.h"
#include "lcd.h"
#include "float16.h"

//#define DEBUG

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbux_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbusInit mrbeeInit
#define mrbusPacketTransmit mrbeePacketTransmit
#endif

#include "mrbus.h"

// MRBee doesn't have a concept of arbitration, so there's no priority level
#ifndef MRBEE
extern uint8_t mrbus_priority;
#endif

uint8_t mrbus_dev_addr = 0;

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t updateXmitInterval=20;
volatile uint16_t screenUpdateDecisecs=0;
volatile uint16_t fastDecisecs=0;
volatile uint8_t scaleTenthsAccum = 0;
uint16_t scaleFactor = 10;
volatile uint8_t status=0;

#define WX_TIMEOUT 180

float wxTemp = NAN;
float wxHumidity = NAN;
uint16_t wxPressure = 0;
uint8_t wxVoltage = 0;
uint8_t wxRSSI = 0;
uint8_t wxAddr = 0;
uint32_t wxAge = 0;

float wxTemp_max;
float wxTemp_min;

#define TH_ALTERNATOR_MAX 8

#define STATUS_READ_INPUTS 0x01
#define STATUS_FAST_ACTIVE 0x02
#define STATUS_FAST_AMPM   0x04
#define STATUS_REAL_AMPM   0x08
#define STATUS_FAST_HOLDING 0x10 // This hold flag indicates we're actually in hold
#define STATUS_FAST_HOLD   0x20  // This flag indicates that we start going into fast in hold
#define STATUS_TEMP_DEG_F  0x40

#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08

#define FAST_MODE (status & STATUS_FAST_ACTIVE)
#define FASTHOLD_MODE (status & STATUS_FAST_HOLDING)


#define EE_ADDR_CONF_FLAGS     0x20

#define CONF_FLAG_FAST_AMPM          0x04
#define CONF_FLAG_REAL_AMPM          0x08
#define CONF_FLAG_FAST_HOLD_START    0x20
#define CONF_FLAG_TEMP_DEG_F         0x40


#define EE_ADDR_FAST_START1_H   0x30
#define EE_ADDR_FAST_START1_M   0x31
#define EE_ADDR_FAST_START1_S   0x32
#define EE_ADDR_FAST_START2_H   0x33
#define EE_ADDR_FAST_START2_M   0x34
#define EE_ADDR_FAST_START2_S   0x35
#define EE_ADDR_FAST_START3_H   0x36
#define EE_ADDR_FAST_START3_M   0x37
#define EE_ADDR_FAST_START3_S   0x38

#define EE_ADDR_FAST_RATIO_H   0x3A
#define EE_ADDR_FAST_RATIO_L   0x3B
#define EE_ADDR_TH_SRC_ADDR    0x3C
#define EE_ADDR_TH_TIMEOUT_L   0x3D
#define EE_ADDR_TH_TIMEOUT_H   0x3E

uint32_t loopCount = 0;

uint8_t hexChars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void blankCursorLine()
{
	lcd_gotoxy(0,2);
	lcd_puts("                    ");
}

void storeConfiguration(uint8_t confStatus)
{
	eeprom_write_byte((uint8_t*)(uint16_t)EE_ADDR_CONF_FLAGS, (confStatus & (STATUS_FAST_AMPM | STATUS_REAL_AMPM | STATUS_FAST_HOLD | STATUS_TEMP_DEG_F)));
}

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t dayOfWeek;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} TimeData;

TimeData realTime;
TimeData fastTime;

void initTimeData(TimeData* t)
{
	t->seconds = t->minutes = t->hours = 0;
	t->dayOfWeek = 0;
	t->year = 2012;
	t->month = t->day = 1;
}

void incrementTime(TimeData* t, uint8_t incSeconds)
{
	uint16_t i = t->seconds + incSeconds;

	while(i >= 60)
	{
		t->minutes++;
		i -= 60;
	}
	t->seconds = (uint8_t)i;
	
	while(t->minutes >= 60)
	{
		t->hours++;
		t->minutes -= 60;
	}
	
	if (t->hours >= 24)
		t->hours %= 24;
}


void FastTimeStartToFlash(TimeData* t, uint8_t whichStart)
{
	whichStart *= 3;
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_H + whichStart), t->hours);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_M + whichStart), t->minutes);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_S + whichStart), t->seconds);			
}
void FlashToFastTimeStart(TimeData* t, uint8_t whichStart)
{
	initTimeData(t);
	whichStart *= 3;
	t->hours = eeprom_read_byte((uint8_t*)EE_ADDR_FAST_START1_H + whichStart);
	t->minutes = eeprom_read_byte((uint8_t*)EE_ADDR_FAST_START1_M + whichStart);
	t->seconds = eeprom_read_byte((uint8_t*)EE_ADDR_FAST_START1_S + whichStart);

	if (t->hours > 23 || t->minutes > 59 || t->seconds > 59)
	{
		initTimeData(t);
		FastTimeStartToFlash(t, whichStart);
	}
}

volatile uint8_t clock_A=0, clock_B=0, debounced_state=0;

uint8_t debounce(uint8_t raw_inputs)
{
  uint8_t delta = raw_inputs ^ debounced_state;   //Find all of the changes
  uint8_t changes;

  clock_A ^= clock_B;                     //Increment the counters
  clock_B  = ~clock_B;

  clock_A &= delta;                       //Reset the counters if no changes
  clock_B &= delta;                       //were detected.

  changes = ~((~delta) | clock_A | clock_B);
  debounced_state ^= changes;
  debounced_state &= 0xFC;
  return(changes & ~(debounced_state));
}

const char* monthNames[13] = { "Unk", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
const uint8_t monthDays[13] = { 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; // Days in each month

typedef enum
{
	SCREEN_MAIN_DRAW = 0,
	SCREEN_MAIN_IDLE = 1,

	SCREEN_DIAG_DRAW = 10,
	SCREEN_DIAG_IDLE = 11,

	SCREEN_ALMNC_DRAW = 20,
	SCREEN_ALMNC_IDLE = 21,
	
	SCREEN_RESET_DRAW = 30,
	SCREEN_RESET_IDLE = 31,
	
	SCREEN_DONT_KNOW = 255

} ScreenState;

#define SOFTKEY_1 0x10
#define SOFTKEY_2 0x20
#define SOFTKEY_3 0x40
#define SOFTKEY_4 0x80

typedef struct
{
	const char* configName;
	ScreenState configScreen;
} ConfigurationOption;

#define NUM_RATIO_OPTIONS  (sizeof(ratioOptions)/sizeof(ConfigurationOption))
#define NUM_CONF_OPTIONS  (sizeof(configurationOptions)/sizeof(ConfigurationOption))

#define isLeapYear(y)  (0 == ((y) % 4))

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	decisecs = 0;
	fastDecisecs = 0;
	scaleTenthsAccum = 0;
	screenUpdateDecisecs = 0;
}


ISR(TIMER1_OVF_vect)
{
	TCNT1 += 0xF3CB;

	if (ticks & 0x01)
		status |= STATUS_READ_INPUTS;

	if (++ticks >= 10)
	{
		ticks = 0;
		if (STATUS_FAST_ACTIVE == (status & (STATUS_FAST_ACTIVE | STATUS_FAST_HOLDING)))
		{
			fastDecisecs += scaleFactor / 10;
			scaleTenthsAccum += scaleFactor % 10;
			if (scaleTenthsAccum > 10)
			{
				fastDecisecs++;
				scaleTenthsAccum -= 10;
			}
			
		}
		decisecs++;
		screenUpdateDecisecs++;
	}
}


volatile uint16_t busVoltageAccum=0;
volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

ISR(ADC_vect)
{
	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/3) / 5 * 1024
        //So multiply by 150, divide by 1024, or multiply by 75 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 75) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 6;
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	} 
	else if ('W' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
		if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	
	}
	else if ('R' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}
	else if ('S' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// WX packet
#ifndef DEBUG
		uint16_t wxTmp1 = (((uint16_t)mrbus_rx_buffer[12])<<8) + (uint16_t)mrbus_rx_buffer[13];
		void *ptr = &wxTmp1;
		float16_t wxFloat16 = *(float16_t*)ptr;
		wxTemp = F16toF32(wxFloat16);
		wxTemp = (wxTemp * 1.8) + 32;  // C to F conversion
		
		if(wxTemp > wxTemp_max)
			wxTemp_max = wxTemp;
		if(wxTemp < wxTemp_min)
			wxTemp_min = wxTemp;

		wxTmp1 = (((uint16_t)mrbus_rx_buffer[10])<<8) + (uint16_t)mrbus_rx_buffer[11];
		ptr = &wxTmp1;
		wxFloat16 = *(float16_t*)ptr;
		wxHumidity = F16toF32(wxFloat16);
#endif

		wxPressure = 0;
		wxVoltage = mrbus_rx_buffer[14];
		wxRSSI = mrbee_rssi;
		wxAddr = mrbus_rx_buffer[MRBUS_PKT_SRC];
		wxAge = 0;
	}


	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
	return;	
}

#define LCD_BACKLIGHT_PIN  PD3
#define BUZZER_PIN PC4

void readDS1302(uint8_t* ds1302Buffer)
{
	ds1302_transact(0xBF, 7, ds1302Buffer);
	realTime.seconds = (ds1302Buffer[0] & 0x0F) + 10 * (((ds1302Buffer[0] & 0x70)>>4));
	realTime.minutes = (ds1302Buffer[1] & 0x0F) + 10 * (((ds1302Buffer[1] & 0x70)>>4));
	realTime.hours = (ds1302Buffer[2] & 0x0F) + 10 * (((ds1302Buffer[2] & 0x30)>>4));			
	realTime.day = (ds1302Buffer[3] & 0x0F) + 10 * (((ds1302Buffer[3] & 0x30)>>4));
	realTime.month = (ds1302Buffer[4] & 0x0F) + ((ds1302Buffer[4] & 0x10)?10:0);
	realTime.year = 2000 + (ds1302Buffer[6] & 0x0F) + 10 * (ds1302Buffer[6]>>4);
}


void init(void)
{
	uint8_t ds1302Buffer[10];	
	// Kill watchdog
    MCUSR = 0;
	wdt_reset();
	wdt_disable();

	DDRD &= ~(0xF2);
	DDRC &= ~(0x10);
	DDRD |= _BV(LCD_BACKLIGHT_PIN);
	DDRC |= _BV(BUZZER_PIN);
	
	PORTD |= _BV(LCD_BACKLIGHT_PIN);
	PORTC &= ~_BV(BUZZER_PIN);
		
	ds1302_init();
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();


	// Setup ADC
	ADMUX  = 0x46;  // AVCC reference; ADC6 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	busVoltageAccum = 0;
	busVoltageCount = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

	// Setup real time data
	initTimeData(&realTime);
	readDS1302(ds1302Buffer);

	if (ds1302Buffer[0] & 0x80)
	{
		// Crap, the clock's been halted, get it started again
		// Set write enable
		ds1302Buffer[0] = 0x00;
		ds1302_transact(0x8E, 1, ds1302Buffer);
		// Clear the clock halting bit
		ds1302Buffer[0] &= 0x7F;
		ds1302_transact(0x80, 1, ds1302Buffer);
	}

	// Clear DS1302 WR enable
	ds1302Buffer[0] = 0x80;
	ds1302_transact(0x8E, 1, ds1302Buffer);	

	initTimeData(&fastTime);
	FlashToFastTimeStart(&fastTime, 0);
	
	status = eeprom_read_byte((uint8_t*)EE_ADDR_CONF_FLAGS);
	
	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	
	updateXmitInterval = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
			| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	updateXmitInterval = max(1, updateXmitInterval);
			
	// If the update interval is garbage, set it to 2 seconds
	if (0xFFFF == updateXmitInterval)
	{
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, 20);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, 0);
		updateXmitInterval = 20;
	}

	wxTemp_max = -INFINITY;
	wxTemp_min = INFINITY;
}

void drawBigTemp(int16_t deciTemp)
{
	uint8_t negative = 0;
	
	if(deciTemp < 0)
	{
		deciTemp *= -1;
		negative = 1;
	}
	else if(deciTemp < 1000)
	{
		lcd_gotoxy(0,0);
		lcd_putc(' ');
		lcd_gotoxy(0,1);
		lcd_putc(' ');
	}
	else
	{
		lcd_putc_big(0, (deciTemp / 1000) % 10);
	}
	
	if(deciTemp >= 100)
	{
		lcd_putc_big(1, (deciTemp / 100) % 10);
		if(negative)
		{
			// Draw minus sign
			lcd_gotoxy(0,0);
			lcd_putc(4);
		}
	}
	else
	{
		if(negative)
		{
			// Draw minus sign
//			lcd_gotoxy(3,0);
//			lcd_putc(4);
			lcd_gotoxy(4,0);
			lcd_putc(4);
		}
		else
		{
			lcd_putc_big(1, 13);
		}
	}

	lcd_putc_big(2, (deciTemp / 10) % 10);
	
	lcd_putc_big(3, deciTemp % 10);
	
	lcd_gotoxy(9,1);
	lcd_putc('.');
	
	lcd_gotoxy(13, 0);
	lcd_putc(0xDF);
	lcd_gotoxy(13, 1);
	lcd_putc('F');
}

void drawSmallTemp(int16_t deciTemp, uint8_t x, uint8_t y)
{
	uint8_t negative = 0;

	if(deciTemp < 0)
	{
		negative = 1;
		deciTemp *= -1;
	}

	lcd_gotoxy(x,y);

	if(deciTemp >= 1000)
	{
		lcd_putc(0x30 + (deciTemp / 1000) % 10);
	}
	else
	{
		lcd_putc(' ');
	}

	x++;
	lcd_gotoxy(x++,y);
	if(deciTemp >=100)
	{
		lcd_putc(0x30 + (deciTemp / 100) % 10);
		if(negative)
		{
			lcd_gotoxy(x-2,y); // Put in 100s spot
			lcd_putc('-');
			negative = 0;
		}
	}
	else
	{
		if(negative)
		{
			lcd_putc('-'); // Put in 10s spot
			negative = 0;
		}
		else
		{
			lcd_putc(' ');
		}
	}
	lcd_gotoxy(x++,y);
	lcd_putc(0x30 + (deciTemp / 10) % 10);
	lcd_gotoxy(x++,y);
	lcd_putc('.');
	lcd_gotoxy(x++,y);
	lcd_putc(0x30 + (deciTemp / 1) % 10);

	lcd_gotoxy(x++,y);
	lcd_putc(0xDF);
	lcd_gotoxy(x++,y);
	lcd_putc('F');
}

void drawRainfall(uint16_t centiRainfall)
{
	lcd_gotoxy(0,2);
	lcd_putc(0x30 + ((centiRainfall / 1000) % 10));
	lcd_gotoxy(1,2);
	lcd_putc(0x30 + ((centiRainfall / 100) % 10));

	lcd_gotoxy(2,2);
	lcd_putc('.');

	lcd_gotoxy(3,2);
	lcd_putc(0x30 + ((centiRainfall / 10) % 10));
	lcd_gotoxy(4,2);
	lcd_putc(0x30 + ((centiRainfall / 1) % 10));
	
	lcd_gotoxy(5,2);
	lcd_putc('"');
}

void drawHumidity(uint8_t humidity)
{
	lcd_gotoxy(8,2);
	lcd_putc(0x30 + ((humidity / 10) % 10));
	
	lcd_gotoxy(9,2);
	lcd_putc(0x30 + (humidity % 10));
	
	lcd_gotoxy(10,2);
	lcd_putc('%');
}

void drawPressure(uint16_t centiPressure)
{
	lcd_gotoxy(13,2);
	lcd_putc(0x30 + ((centiPressure / 1000) % 10));
	
	lcd_gotoxy(14,2);
	lcd_putc(0x30 + ((centiPressure / 100) % 10));
	
	lcd_gotoxy(15,2);
	lcd_putc('.');
	
	lcd_gotoxy(16,2);
	lcd_putc(0x30 + ((centiPressure / 10) % 10));

	lcd_gotoxy(17,2);
	lcd_putc(0x30 + (centiPressure % 10));

	lcd_gotoxy(18,2);
	lcd_putc('i');

	lcd_gotoxy(19,2);
	lcd_putc('n');
}

void drawWind(uint8_t windMph, uint16_t direction)
{
/*	lcd_putc_big(0, (deciTemp / 1000) % 10);*/
/*	lcd_putc_big(1, (deciTemp / 100) % 10);*/
/*	lcd_putc_big(2, (deciTemp / 10) % 10);*/
/*	lcd_putc_big(3, deciTemp % 10);*/
/*	*/
/*	lcd_gotoxy(1, 13);*/
/*	lcd_putc('F');*/
}


void drawSplashScreen()
{
	lcd_setup_bigdigits();
	lcd_gotoxy(0,0);
//            00000000001111111111
//            01234567890123456789	
	lcd_gotoxy(0,0);
	lcd_puts("  Weather  Display  ");
	lcd_gotoxy(0,1);
	lcd_puts("MRBus Enabled   v2.0");
	lcd_gotoxy(0,2);
	lcd_puts("2024 Iowa Scaled Eng");
	lcd_gotoxy(0,3);
	lcd_puts("  www.iascaled.com  ");
	_delay_ms(2000);
	lcd_clrscr();	
}

void drawSoftKeys(char* key1Text, char* key2Text, char* key3Text, char* key4Text)
{
	uint8_t i;

	lcd_gotoxy(0,4);
	for(i=0; i<20; i++)
		lcd_putc(' ');

	lcd_gotoxy(0,3);
	lcd_puts(key1Text);
	lcd_gotoxy(5,3);
	lcd_puts(key2Text);
	lcd_gotoxy(10,3);
	lcd_puts(key3Text);
	lcd_gotoxy(15,3);
	lcd_puts(key4Text);
}

int main(void)
{
//	uint8_t ds1302Buffer[10];
	uint8_t buttonsPressed=0;
	
	int16_t deciTemperature = 0;
	uint16_t centiRainfall = 0;
	uint8_t humidity = 0;
	uint16_t centiPressure = 0;

	#ifdef DEBUG
	wxTemp = 3730;
	#endif
	
	ScreenState screenState = SCREEN_MAIN_DRAW;
	// Application initialization
	init();

	DDRD &= ~(0xF0);

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();
	
	// MRBee doesn't have a concept of arbitration, so there's no priority level
#ifndef MRBEE
	mrbus_priority = 1;  // We're a clock, highest priority
#endif

	drawSplashScreen();

	sei();	

	loopCount = 0;

	while (1)
	{
		loopCount++;
#ifdef MRBEE
		mrbeePoll();
#endif
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();

		if (status & STATUS_READ_INPUTS)
		{
			status &= ~(STATUS_READ_INPUTS);
			buttonsPressed = debounce((PIND & 0xF8) | (PINC & _BV(PC4)>>2));
		}

		switch(screenState)
		{
		
			case SCREEN_MAIN_DRAW:
				lcd_clrscr();
				lcd_gotoxy(16,1);
				
				if(isnan(wxTemp))
				{
					drawBigTemp(-999);
				}
				else
				{
					// Convert 1/16K to deci Fahrenheit
					// 320 + (((T - 4370) * 9 * 10) / (5 * 16))
					deciTemperature = wxTemp * 10;
					drawBigTemp(deciTemperature);
				}

				centiRainfall = 0;
				drawRainfall(centiRainfall);

				humidity = wxHumidity;
				drawHumidity(humidity);
				
				// WARNING!!! Very ugly code (for a microcontroller) ahead!
				// Donnellson = 680' = 207m
				// http://keisan.casio.com/exec/system/1224575267
				// P0 = P*(1 - (0.0065h / (0.0065h + T)))^-5.257
				// h = meters, P = hPa, T = K
				// 0.0065h = 1.3455
				// float tempPressure = wxPressure * pow(1.0 - (1.3455 / (1.3455 + wxTemp)),-5.257);

				// OK, I give up.  Just cheat.  1 hPa = 0.0296133971008484 inHg or 33.7685 hPa = 1 inHg or 0.337685 hPa = 0.01 inHg
				centiPressure = wxPressure * 3;
				drawPressure(centiPressure);

/*				lcd_gotoxy(16,0);*/
/*				lcd_putc(0x30 + ((wxPressure / 1000) % 10));*/
/*				lcd_putc(0x30 + ((wxPressure / 100) % 10));*/
/*				lcd_putc(0x30 + ((wxPressure / 10) % 10));*/
/*				lcd_putc(0x30 + ((wxPressure / 1) % 10));*/
				
				drawSoftKeys(" DIAG",  "", "", "ALMNC");

				// Must come after drawSoftKeys since it stomps on the middle ones
				if(wxAge > WX_TIMEOUT)
				{
					lcd_gotoxy(6,3);
					lcd_puts("*STALE*");
				}

				
				screenState = SCREEN_MAIN_IDLE;
				break;

			case SCREEN_MAIN_IDLE:
				if(PIND & _BV(LCD_BACKLIGHT_PIN))
				{
					if (SOFTKEY_1 & buttonsPressed)
					{
						screenState = SCREEN_DIAG_DRAW;
					}
					else if (SOFTKEY_4 & buttonsPressed)
					{
						screenState = SCREEN_ALMNC_DRAW;
					}
				}
				else
				{
					if((SOFTKEY_1 | SOFTKEY_2 | SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
					{
						//  Wake backlight on any key press
						PORTD |= _BV(LCD_BACKLIGHT_PIN);
					}
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;
				
			case SCREEN_DIAG_DRAW:
				lcd_clrscr();
				lcd_gotoxy(0,0);
				lcd_puts("Battery:");
				lcd_gotoxy(9,0);
				lcd_putc(0x30 + ((wxVoltage / 10) % 10));
				lcd_gotoxy(10,0);
				lcd_putc('.');
				lcd_gotoxy(11,0);
				lcd_putc(0x30 + (wxVoltage % 10));
				lcd_gotoxy(12,0);
				lcd_putc('V');

				lcd_gotoxy(0,1);
				lcd_puts("RSSI: -");
				// Value can be 0x64 according to XBee manual.  Might overflow code below.
				lcd_gotoxy(7,1);
				lcd_putc(0x30 + ((wxRSSI / 10) % 10));
				lcd_gotoxy(8,1);
				lcd_putc(0x30 + ((wxRSSI / 1) % 10));
				lcd_gotoxy(9,1);
				lcd_puts("dBm");
				
				lcd_gotoxy(16,1);
				lcd_puts("0x");
				lcd_gotoxy(18,1);
				lcd_putc(hexChars[wxAddr >> 4]);
				lcd_gotoxy(19,1);
				lcd_putc(hexChars[wxAddr & 0xF]);
				
				uint8_t H = (wxAge-1) / 3600;
				uint8_t M = ((wxAge-1) - (H * 3600)) / 60;
				uint8_t S = ((wxAge-1) - (M * 60));
				lcd_gotoxy(0,2);
				lcd_puts("Pkt Age:");
				lcd_gotoxy(9,2);
				lcd_putc(0x30 + ((H / 10) % 10));
				lcd_gotoxy(10,2);
				lcd_putc(0x30 + ((H / 1) % 10));
				lcd_gotoxy(11,2);
				lcd_putc(':');
				lcd_gotoxy(12,2);
				lcd_putc(0x30 + ((M / 10) % 10));
				lcd_gotoxy(13,2);
				lcd_putc(0x30 + ((M / 1) % 10));
				lcd_gotoxy(14,2);
				lcd_putc(':');
				lcd_gotoxy(15,2);
				lcd_putc(0x30 + ((S / 10) % 10));
				lcd_gotoxy(16,2);
				lcd_putc(0x30 + ((S / 1) % 10));
				
				
				drawSoftKeys(" BACK",  "", "", "LIGHT");
				screenState = SCREEN_DIAG_IDLE;
				break;
				
			case SCREEN_DIAG_IDLE:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_MAIN_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_MAIN_DRAW;
					PORTD &= ~_BV(LCD_BACKLIGHT_PIN);
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;
				
			case SCREEN_ALMNC_DRAW:
				lcd_clrscr();
				
				lcd_gotoxy(0,0);
				lcd_puts("Max Temp:");
				if(-INFINITY == wxTemp_max)
				{
					lcd_gotoxy(10,0);
					lcd_puts("---.-");
				}
				else
				{
					deciTemperature = wxTemp_max * 10;
					drawSmallTemp(deciTemperature, 10, 0);
				}

				lcd_gotoxy(0,1);
				lcd_puts("Min Temp:");
				if(INFINITY == wxTemp_min)
				{
					lcd_gotoxy(10,1);
					lcd_puts("---.-");
				}
				else
				{
					deciTemperature = wxTemp_min * 10;
					drawSmallTemp(deciTemperature, 10, 1);
				}

				drawSoftKeys(" RST",  "", "", "BACK");
				screenState = SCREEN_ALMNC_IDLE;
				break;
				
			case SCREEN_ALMNC_IDLE:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_RESET_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_MAIN_DRAW;
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case SCREEN_RESET_DRAW:
				lcd_clrscr();
				lcd_gotoxy(6,0);
				lcd_puts("Almanac");
				lcd_gotoxy(3,1);
				lcd_puts("Confirm Reset?");
				drawSoftKeys("",  " YES", "  NO", "");
				screenState = SCREEN_RESET_IDLE;
				break;

			case SCREEN_RESET_IDLE:
				if (SOFTKEY_2 & buttonsPressed)
				{
					wxTemp_max = -99.9;
					wxTemp_min = 199.9;
					screenState = SCREEN_ALMNC_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					screenState = SCREEN_ALMNC_DRAW;
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			default:
				lcd_gotoxy(0,1);
				lcd_puts("Code off in lalaland");
				lcd_gotoxy(0,2);
				lcd_puts("Call Nathan");

				break;		
		
		}

		if (screenUpdateDecisecs >= 10)
		{
/*			// Reading optimizer*/
/*			// If we don't know the date or if we're in the range where we're at risk of */
/*			// changing dates*/
/*			readDS1302(ds1302Buffer);*/
/*			*/
/*			switch(screenState)*/
/*			{*/
/*				case SCREEN_MAIN_IDLE:*/
/*				case SCREEN_MAIN_UPDATE_TIME:*/
/*					screenState = SCREEN_MAIN_UPDATE_TIME;*/
/*					if (0xA5 == colon)*/
/*						colon = ' ';*/
/*					else*/
/*						colon = 0xA5;*/
/*					*/
/*					lcd_gotoxy(7,0);*/
/*					lcd_putc(colon);*/
/*					lcd_gotoxy(7,1);*/
/*					lcd_putc(colon);*/
/*					break;*/
/*				*/
/*				case SCREEN_CONF_DIAG_IDLE:*/
/*					// Get loop iterations here*/
/*					screenState = SCREEN_CONF_DIAG_SETUP;*/
/*					break;*/
/*				*/
/*				case SCREEN_MAIN_DRAW:*/
/*				default:*/
/*					break;*/
/*			}*/
			
			screenUpdateDecisecs -= 10;
				
			loopCount = 0;

/*			deciTemperature += 100;*/
/*			if(deciTemperature > 1500)*/
/*				deciTemperature = -500;*/
/*			humidity++;*/
/*			centiPressure++;*/
			
			wxAge++;

#ifdef DEBUG
			wxTemp += 32;
			wxTemp_max = wxTemp;
			wxTemp_min = wxTemp;
#endif

			switch(screenState)
			{
				case SCREEN_MAIN_IDLE:
					screenState = SCREEN_MAIN_DRAW;
					break;
				case SCREEN_DIAG_IDLE:
					screenState = SCREEN_DIAG_DRAW;
					break;
				case SCREEN_ALMNC_IDLE:
					screenState = SCREEN_ALMNC_DRAW;
					break;
				default:
					break;
			}
		}		

		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
//			uint8_t bus_countdown;

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

#ifndef MRBEE
			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
				if (mrbus_state & MRBUS_RX_PKT_READY) 
					PktHandler();
			}
#endif
		}
	}
}



