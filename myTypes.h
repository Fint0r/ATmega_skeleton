/*
 * myTypes.h
 *
 * Created: 2019. 10. 10. 11:24:52
 *  Author: Fintor József
 */ 


/* This file contains typedefs and not changeable macros */


#ifndef MYTYPES_H_
#define MYTYPES_H_



/* ================================= INCLUDES ================================= */
#include <avr/io.h>
#ifndef F_CPU
	#define F_CPU 8000000
#endif /* F_CPU */
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <compat/twi.h>
#include <string.h>



/* ================================= TYPEDEFS ================================= */
typedef unsigned char uint8;
typedef unsigned int uint16;
typedef char sint8;
typedef int sint16;
typedef float float32;



/* ================================= MACROS ================================= */
#define UCHAR_MAX		255
#define SCHAR_MIN		-128
#define SCHAR_MAX		127
#define UINT_MAX		65535
#define SINT_MIN		-32768
#define SINT_MAX		32767

#define MATRIX_NO_PRESS 222

#define BUTTON_HOLD 222

#define BUT0	1	
#define BUT1	2
#define BUT2	4
#define BUT3	8
#define BUT4	16

#define HIGH	1
#define LOW		0

#define ON		1
#define OFF		0

#define TRUE	1
#define FALSE	0

#define LF_ASCII	10
#define CR_ASCII	13

#define FINISHED		1
#define NOT_FINISHED	0

#define CALLBACKS
#define GLOBALS


/* ================================= LCD MACROS ================================= */
#define LCD_CMD_DDR (DDRF)
#define LCD_DATA_DDR (DDRE)
#define LCD_CMD_PORT (PORTF)
#define LCD_DATA_PORT (PORTE)
#define LCD_DATA_PIN (PINE)
#define LCD_RS (PF1)
#define LCD_RW (PF2)
#define LCD_EN (PF3)
#define LCD_BUSY_PIN (PE7)

#define LCD_E	2
#define LCD_CUR	1
#define LCD_BL	0

#define SOR_1	(128 + 0)
#define SOR_2	(128 + 64)
#define SOR_3	(128 + 16)
#define SOR_4	(128 + 80)



/* ================================= SPI MACROS ================================= */
#define DDR_SPI			DDRB
#define DD_MOSI			PB2
#define DD_MISO			PB3
#define DD_SCK			PB1
#define DD_SS			PB0

#define MASTER			0
#define SLAVE			1
#define SPI_OFF			2


/* ================================= RS 485 MACROS ================================= */
#define UART1_DDR	(DDRD)
#define UART1_PORT	(PORTD)
#define UART1_TX	(PD3)
#define UART1_RX	(PD7)

#define RS_R_PIN	(PD2)		//in
#define RS_RE_PIN	(PC7)		//out
#define RS_DE_PIN	(PE2)		//out
#define RS_D_PIN	(PD3)		//out


#define RS_DE_DDR	(DDRE)
#define RS_DE_PORT	(PORTE)
#define RS_DE_PIN	(PE2)
#define RS485_OFF	2


/* ================================= TWI MACROS ================================= */
#define SCL_CLK 100000		/* 100kHz CLOCK */
/* CONFIG */
#define WRITE 0				/* !WR */
#define READ 1				/*  RD */
#define RTC_ADDR_READ 0xA3
#define RTC_ADDR_WRITE 0xA2
/* SET ADDRESSES */
#define SECS_ADDR 0x02		/* WRITE IN VALUES: 0-59 */
#define MINS_ADDR 0x03		/* WRITE IN VALUES: 0-59 */
#define HOURS_ADDR 0x04		/* WRITE IN VALUES: 0-23 */
#define DAYS_ADDR 0x05		/* WRITE IN VALUES: 1-31 */
#define WEEKDAYS_ADDR 0x06	/* WRITE IN VALUES: 0-6 */
#define MONTHS_ADDR 0x07	/* WRITE IN VALUES: 1-12 */
#define YEARS_ADDR 0x08		/* WRITE IN VALUES: 0-99 */

#endif /* MYTYPES_H_ */