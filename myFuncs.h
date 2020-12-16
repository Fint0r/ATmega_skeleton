/*
 * myFuncs.h
 *
 * Created: 2019. 10. 10. 11:36:36
 *  Author: Fintor József
 */ 


/* This file contains the changeable macros and function declarations to librarys */


#ifndef MYFUNCS_H_
#define MYFUNCS_H_


/* ================================= INCLUDES ================================= */
#include "myTypes.h"

/* ================================= COMPILE TIME SETTINGS ================================= */
#define LEDS
#define SEVSEG
#define MATRIX
#define BUTTONS
#define LCD
#define SERIAL
#define MYADC
#define MYSPI			SPI_OFF						/* MASTER / SLAVE / SPI_OFF */
#define RS485			MASTER						/* MASTER / SLAVE / RS485_OFF */
#define TWI
#define RTC_SET_TIME




/* BUTTONS SETTINGS */
#define DEBOUNCE	ON							/* OFF / ON */


/* LCD SETTINGS */			
#define LCD_ENABLE	ON							/* OFF / ON */
#define LCD_CURSOR	OFF							/* OFF / ON */
#define LCD_BLINK	OFF							/* OFF / ON */

#define LCD_INIT_MESSAGE OFF				/* OFF / ON */
#define LCD_INIT_MESSAGE_DELAY ON				/* OFF / ON */
#define LCD_INIT_MESSAGE_DELAY_TIME 3000		/* in ms */


/* SERIAL SETTINGS */
#define BAUD 9600								/* BAUD = bit/s */

#define SERIAL0_INIT_MESSAGE ON				/* OFF / ON */
#define SERIAL0_INIT_MESSAGE_DELAY OFF			/* OFF / ON */
#define SERIAL0_INIT_MESSAGE_DELAY_TIME 2000	/* in ms */
#define SERIAL1_INIT_MESSAGE OFF				/* OFF / ON */
#define SERIAL1_INIT_MESSAGE_DELAY OFF			/* OFF / ON */
#define SERIAL1_INIT_MESSAGE_DELAY_TIME 2000	/* in ms */


/* RTC SETTINGS */
#define RTC_HOURS		23
#define RTC_MINS		59
#define RTC_SECS		55
 
/* ADC SETTINGS */
#define ADC_SAMPLING 5							/* sample times. Calculating the average value */


/* ================================= MACROS ================================= */
#define UBRR (((F_CPU)/16/BAUD) - 1)





/* ================================= FUNCTION DECLARATIONS ================================= */
void init(void);
uint8 dec_to_bcd(uint8 dec);
uint8 bcd_to_dec(uint8 bcd);



#ifdef LEDS
/* ================================= LEDS ================================= */
	void showleds(uint8 leds);
#endif	/* LEDS */



#ifdef SEVSEG
/* ================================= 7SEG ================================= */
	void showseg(uint16 sevseg);	
#endif	/* SEVSEG */



#ifdef MATRIX
/* ================================= MATRIX ================================= */
	uint8 readmatrix(void);
#endif	/* MATRIX */



#ifdef BUTTONS
/* ================================= BUTTONS ================================= */
	uint8 readbuts(void);
#endif	/* BUTTONS */



#ifdef LCD
/* ================================= LCD ================================= */
	void LCD_init(void);
	void LCD_busy(void);
	void LCD_clock(void);
	void LCD_cmd(uint8 cmd);
	void LCD_write(char* str);
	void LCD_data(uint8 data);
	void LCD_creatctg(void);
	void LCD_lineselect(uint8 line);
	void LCD_text_with_pos(char* text, uint8 sor);
	void LCD_char_with_pos(char text, uint8 sor, uint8 oszlop);
	void LCD_clear(void);
	void LCD_rshift(void);
	void LCD_lshift(void);
	void LCD_init_message(void);
#endif	/* LCD */



#ifdef SERIAL
/* ================================= SERIAL ================================= */
	void ser_init(void);
	void ser0_write_char(char betu);
	void ser0_write_str(char *str);
	char ser0_read_char(void);
	void ser0_read_str(void);
	void ser0_init_message(void);
	void ser0_writeline(char *str);
	
	void ser1_write_char(char betu);
	void ser1_write_str(char *str);
	char ser1_read_char(void);
	void ser1_read_str(void);
	void ser1_init_message(void);
	void ser1_writeline(char *str);
#endif	/* SERIAL */



#ifdef MYADC
/* ================================= ADC ================================= */
	void ADC_init(void);
	void ADC_start(void);
#endif	/* ADC */



#if MYSPI == MASTER
/* ================================= SPI MASTER ================================= */
	void SPI_init_master(void);
	void SPI_write_char(char betu);
	void SPI_write_str(char *str);
#endif	/* SPI_MASTER */



#if MYSPI == SLAVE
/* ================================= SPI SLAVE ================================= */
	void SPI_init_slave(void);
	void SPI_write_char(char betu);
	void SPI_write_str(char *str);
#endif	/* SPI_SLAVE */


#if RS485 == MASTER
/* ================================= RS 485 MASTER ================================= */
	void rs_init_master(void);
	void rs_init_slave(void);
	void rs_write_char(char betu);
	void rs_write_str(char *str);
	void rs_writeline(char *str);
#endif	/* RS485_MASTER */


#if RS485 == SLAVE
/* ================================= RS 485 MASTER ================================= */
	void rs_init_slave(void);
	void rs_init_master(void);
	void rs_write_char(char betu);
	void rs_write_str(char *str);
	void rs_writeline(char *str);
#endif	/* RS485_SLAVE */



#ifdef TWI
/* ================================= TWI ================================= */
	void TWI_init(void);
	uint8 TWI_start(uint8 address);
	void TWI_stop(void);
	uint8 TWI_rep_start(uint8 address);
	uint8 TWI_start_wait(uint8 address);
	uint8 TWI_write(uint8 data);
	uint8 TWI_read_ACK(void);
	uint8 TWI_read_NACK(void);
	void RTC_Write(uint8 address, uint8 data_reg, uint8 data);
	uint8 RTC_Read(uint8 address, uint8 data_reg);
	void RTC_write_start_data(void);
#endif	/* TWI */
#endif /* MYFUNCS_H_ */