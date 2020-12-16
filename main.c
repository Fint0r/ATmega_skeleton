/*
 * Skeleton.c
 *
 * Created: 2019. 10. 10. 11:20:25
 * Author : Fintor József
 */


/* This file contains the main function, callback functions and interrupt functions */


/* ================================= INCLUDES ================================= */
#include "myTypes.h"
#include "myFuncs.h"

/* ================================= MACROS ================================= */
#define SERIAL0_BUFF	50
#define SERIAL1_BUFF	50
#define CONVERT_BUFFER	20
#define SPI_BUFFER		50


#define RS_ON							/* if its not defined the UART callback function works as normal  */
#define RS_START_BYTE			0x55	/* indicates the start of the frame */
#define RS_STOP_BYTE			0xC8	/* indicates the end of the frame */
#define RS_SLAVE_ID				0x20	/* this is the slave id */

#define RS_CMD_SEND_DATA		0x02	/* this command should be send if we sending data to the slave */
#define RS_CMD_PASS_TOKEN		0xAA	/* this command can pass the token to the destination node */
#define RS_MY_ID				0x10	/* this is my id */

/* ================================= TYPEDEFS ================================= */




/* ================================= GLOBALS ================================= */
#ifdef GLOBALS

char buffer[CONVERT_BUFFER] = {0};			/* float/integer to string convert buffer */


#ifdef SERIAL
	char serial0[SERIAL0_BUFF] = "";			/* this array contains the received UART0 message */
	char serial1[SERIAL1_BUFF] = "";			/* this array contains the received UART1 message */
	volatile uint8 ser0_rx_compl = NOT_FINISHED;			/* FLAG     set to 1 if receive completed ( looking for +lf in ascii) */
	volatile uint8 ser1_rx_compl = NOT_FINISHED;			/* FLAG     set to 1 if receive completed ( looking for +lf in ascii) */
#endif	/* SERIAL */


#ifdef MYADC
	float32 temperature = 0;					/* LM35 temperature */
#endif	/* MYADC */


#if MYSPI != SPI_OFF
	char SPI_message[SPI_BUFFER] = "";
	volatile uint8 SPI_rx_compl = NOT_FINISHED;			/* FLAG     set to 1 if receive completed ( looking for LF in ascii) */
#endif	/* MYSPI */


#ifdef TWI
	char time_str[10] = "";
	uint8 secs = 0;
	uint8 mins = 0;
	uint8 hours = 0;	
#endif /* TWI */

#ifdef RS485
	uint8 rs_token = 0;			/* frame can only be sent if token is not 0 */
#endif /* RS485 */

#endif /* GLOBALS */
/* ================================= FUNCTION DECLARATIONS ================================= */
void timer_init(void);
void RTC_str_read(void);
void rs_send_frame(uint8 id, uint8 cmd, char* data);
void rs_pass_token(uint8 id);


/* ================================= CALLABLE APIS =================================
	showleds();
	showseg();				must called periodically to use all segments
	readmatrix();			retVal: number pressed, * = 10, # = 11, MATRIX_NO_PRESS
	readbuts();				retVal: PING. use: BUTn  macros, n = 0...4
	
	LCD_write();			inVal: string. print string from cursor current position
	LCD_lineselect();		inVal: 1..4.  cursor moves to the selected line first character
	LCD_char_with_pos();	inVal: (character, line, column)
	LCD_text_with_pos();	inVal: (string, line)
	LCD_rshift();			shifts the entire screen to right
	LCD_lshift();			shifts the entire screen to left
	LCD_clear();			clears the screen and return the cursor to the 1.1 position
	
	ser0_write_char();		inVal: ascii  ('q')
	ser0_write_str();		inVal: string  ("asd")
	ser0_writeline();		inVal: string ("asd")
	
	ser1_write_char();
	ser1_write_str();
	ser1_writeline();
	
	ADC_start();			starts the ADC
	dtostrf();				convert float to string		inVal: (float, digit, precision, buffer);     buffer: array, precision: .xxx = 3
	itoa();					convert integer to string	inVal: (integer, buffer, 10);	10: decimal, 2: binary
	
	SPI_write_char();		inVal: ascii ('q')
	SPI_write_str();		inVal: string ("asd")
	
	rs_write_char();		inVal: ascii  ('q')			!!!   RS 485 using serial1 callback function and variables!   !!!
	rs_write_str();			inVal: string  ("asd")
	rs_writeline();			inVal: string ("asd")
	
	RTC_str_read();			fill time_str array with time from RTC
	RTC_Read();				inVal: (SLAVE_ADDR, REGISTER_ADDR)		retVal: uint8, register data
	RTC_Write();			inVal: (SLAVE_ADDR, REGISTER_ADDR, REGISTER_DATA)
	

	rs_send_frame();		inVal: (SLAVE_ID, COMMAND, data_array)
	rs_pass_token();		inVal: (SLAVE_ID)
*/



/* ================================= MAIN ================================= */
int main(void)
{
	/* this function inits the defined apis */
	init();
	timer_init();
	
	
	/* global interrupts enable */
	sei();
	
	char rs_data[] = "Test message.";
	rs_send_frame(RS_SLAVE_ID, RS_CMD_SEND_DATA, rs_data);
	
/* ================================= MAIN LOOP ================================= */
    while (TRUE) 
    {
		
	}
}



/* ================================= CALLBACK FUNCTIOINS ================================= */
#ifdef CALLBACKS

#ifdef SERIAL
	ISR(USART0_RX_vect)
	{
		static uint8 buff_index = 0;
		static char s[SERIAL0_BUFF] = {0};		/* temp array */
	
		s[buff_index] = UDR0;		/* read 1 byte */
	
		if(s[buff_index] == LF_ASCII)		/* if LF received save the array to global array and reset the temp array */
		{
			buff_index = 0;
			ser0_rx_compl = FINISHED;		/* set flag */
		
			/* save data to global array */
			for(uint8 i = 0; i != (sizeof(serial0)/sizeof(serial0[0])); i++ )
				serial0[i] = s[i];
		
			/* clear the temp array */
			for(uint8 i = 0; i != (sizeof(s)/sizeof(s[0])); i++ )
				s[i] = 0;
		}
		else
		{
			ser0_rx_compl = NOT_FINISHED;
			buff_index++;
		}
	}

#ifdef RS_ON
	ISR(USART1_RX_vect)
	{
		static uint8 buff_index = 0;
		static char s[SERIAL1_BUFF] = {0};		/* temp array */
		
		s[buff_index] = UDR1;		/* read 1 byte */
		
		if((s[0] == RS_START_BYTE) && s[1] == RS_MY_ID)		/* if LF received save the array to global array and reset the temp array */
		{
			if(s[2])
			{
				switch(s[2])
				{
					/* handle commands */
					case RS_CMD_PASS_TOKEN: rs_token = 1; break;
					case RS_CMD_SEND_DATA: break;
				}
			}
		}
		
		buff_index++;
		
		if(s[buff_index-1] == RS_STOP_BYTE)
		{
			s[buff_index-1] = 0;
			
			if((s[0] == RS_START_BYTE) && s[1] == RS_MY_ID)
			{
				/* clear serial1 array */
				for(uint8 i = 0; i != (sizeof(serial1)/sizeof(serial1[0])); i++ )
				serial1[i] = 0;
				
				/* save data to global array */
				for(uint8 i = 3; i != (sizeof(s)/sizeof(s[0])); i++ )
				serial1[i] = s[i];

				ser0_rx_compl = 1;
			}
			
			/* clear the temp array */
			for(uint8 i = 0; i != (sizeof(s)/sizeof(s[0])); i++ )
			s[i] = 0;
			
			buff_index = 0;
		}
	}
	
	
#else	/* RS232/485 is not used so this callback function is used as normal */
ISR(USART1_RX_vect)
{
	static uint8 buff_index = 0;
	static char s[SERIAL1_BUFF] = {0};		/* temp array */
	
	s[buff_index] = UDR1;		/* read 1 byte */
	
	if(s[buff_index] == LF_ASCII)		/* if LF received save the array to global array and reset the temp array */
	{
		buff_index = 0;
		ser1_rx_compl = FINISHED;		/* set flag */
		
		/* save data to global array */
		for(uint8 i = 0; i != (sizeof(serial1)/sizeof(serial1[0])); i++ )
			serial1[i] = s[i];
		
		/* clear the temp array */
		for(uint8 i = 0; i != (sizeof(s)/sizeof(s[0])); i++ )
			s[i] = 0;
	}
	else
	{
		ser1_rx_compl = NOT_FINISHED;
		buff_index++;
	}
}
#endif	/* RS_ON */
#endif	/* SERIAL */


#ifdef MYADC
	ISR(ADC_vect)
	{
		static uint8 sample = 0;
		static float32 v = 0;
	
		v += ADC;

		if(sample == (ADC_SAMPLING - 1))
		{
			v = (v / (float32) ADC_SAMPLING);

			/* convert ADC value to temperature */
			temperature = ((v * (float32)4.096) / (float32)1024 ) * 100;			/* ref voltage: 4.096 */
		
			sample = 0;
			v = 0;
		}
		else
			sample++;
	}
#endif	/* MYADC */


#if MYSPI == SLAVE
	ISR(SPI_STC_vect)
	{
		static uint8 buff_index = 0;
		static char s[SPI_BUFFER] = {0};
	
		s[buff_index] = SPDR;
	
		if(s[buff_index] == LF_ASCII)
		{
			/* save data to global array */
			for(uint8 i = 0; i != (sizeof(s)/sizeof(s[0])); i++ )
				SPI_message[i] = s[i];
		
			/* clear the temp array */
			for(uint8 i = 0; i != (sizeof(s)/sizeof(s[0])); i++ )
				s[i] = 0;
		
			SPI_rx_compl = FINISHED;
			buff_index = 0;
		}
		else
		{
			buff_index++;
			SPI_rx_compl = NOT_FINISHED;
		}	
	}
#endif	/* MYSPI */

#endif	/* CALLBACKS */


/* ================================= INTERRUPTS ================================= */





/* ================================= FUNCTION DEFINITIONS ================================= */
void timer_init(void)
{
	
}

void RTC_str_read(void)
{
	char temp[20] = "";
	strcpy(time_str, "");
	secs = bcd_to_dec(RTC_Read(RTC_ADDR_WRITE, SECS_ADDR) & 0x7F);
	mins = bcd_to_dec(RTC_Read(RTC_ADDR_WRITE, MINS_ADDR) & 0x7F);
	hours = bcd_to_dec(RTC_Read(RTC_ADDR_WRITE, HOURS_ADDR) & 0x3F);
	itoa(hours, temp, 10);
	if(hours < 10) strcat(time_str, "0");
	strcat(time_str, temp);
	strcat(time_str, ":");
	itoa(mins, temp, 10);
	if(mins < 10) strcat(time_str, "0");
	strcat(time_str, temp);
	strcat(time_str, ":");
	itoa(secs, temp, 10);
	if(secs < 10) strcat(time_str, "0");
	strcat(time_str, temp);
}

void rs_send_frame(uint8 id, uint8 cmd, char* data)
{
	if(rs_token)
	{
		rs_init_master();
		rs_write_char(RS_START_BYTE);
		rs_write_char(id);
		rs_write_char(cmd);
		rs_write_str(data);
		rs_write_char(RS_STOP_BYTE);
		rs_init_slave();
	}
}

void rs_pass_token(uint8 id)
{
	if(rs_token)
	{
		rs_init_master();
		rs_write_char(RS_START_BYTE);
		rs_write_char(id);
		rs_write_char(RS_CMD_PASS_TOKEN);
		rs_write_char(RS_STOP_BYTE);
		rs_init_slave();
		rs_token = 0;
	}
}