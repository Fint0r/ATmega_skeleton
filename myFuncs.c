/*
 * myFuncs.c
 *
 * Created: 2019. 10. 10. 11:36:46
 *  Author: Fintor József
 */ 


/* This file contains the function definitions to librarys */


/* ================================= INCLUDES ================================= */
#include "myTypes.h"
#include "myFuncs.h"



/* ================================= FUNCTION DEFINITIONS ================================= */
void init(void)
{
	#ifdef LEDS
		DDRB |= 0xF0;
		DDRD |= 0xF0;
	#endif	/* LEDS */
	
	
	#ifdef SEVSEG
		DDRA |= 0xFF;
	#endif	/* SEVSEG */
	
	
	#ifdef MATRIX
		DDRC |= 0b01111000;
		DDRC &= 0b11111000;
	#endif	/* MATRIX */
	
	
	#ifdef BUTTONS
		DDRG &= 0b11100000;
	#endif	/* BUTTONS */
	
	
	#ifdef LCD
		LCD_init();
	#endif	/* LCD */
	
	
	#ifdef SERIAL
		ser_init();
	#endif	/* SERIAL */
	
	
	#ifdef MYADC
		DDRG &= (0 << PG0);		/* set LM35 pin to input */
		ADC_init();
	#endif	/* ADC */


	#if MYSPI == MASTER
		SPI_init_master();
	#endif 	/* SPI_MASTER */
	
	
	#if MYSPI == SLAVE
		SPI_init_slave();
	#endif 	/* SPI_SLAVE */
	
	
	#if RS485 == MASTER
		UART1_DDR |= (1 << UART1_TX);
		UART1_DDR &= (0 << UART1_RX);
		
		DDRD &= (0 << RS_R_PIN);
		DDRC |= (1 << RS_RE_PIN);
		DDRE |= (1 << RS_DE_PIN);
		DDRD |= (1 << RS_D_PIN);
		/*rs_init_master();*/
		rs_init_slave();
	#endif	/* RS_485_MASTER */
	
	
	#if RS485 == SLAVE
		UART1_DDR |= (1 << UART1_TX);
		UART1_DDR &= (0 << UART1_RX);
		
		DDRD &= (0 << RS_R_PIN);
		DDRC |= (1 << RS_RE_PIN);
		DDRE |= (1 << RS_DE_PIN);
		DDRD |= (1 << RS_D_PIN);
		/*rs_init_slave();*/
		rs_init_slave();
	#endif	/* RS_485_SLAVE */
	
	
	#ifdef TWI
		TWI_init();
		#ifdef RTC_SET_TIME
			RTC_write_start_data();
		#endif	/* RTC_SET_TIME */
	#endif	/* TWI */
}

uint8 dec_to_bcd(uint8 dec)
{
	uint8 bcd = 0;
	bcd = ((((dec/10)%10) << 4) + (dec%10 & 0x0F));
	return bcd;
}

uint8 bcd_to_dec(uint8 bcd)
{
	uint8 dec = 0;
	dec = ((bcd & 0xF0) >> 4) *10 + (bcd & 0x0F);
	return dec;
}


#ifdef LEDS
/* ================================= LEDS ================================= */
	void showleds(uint8 leds)
	{
		PORTD=leds;
		PORTB=(leds<<4);
	}
#endif	/* LEDS */



#ifdef SEVSEG
/* ================================= 7SEG ================================= */
	void showseg(uint16 sevseg)
	{
		static uint8 i = 1;

		switch(i)
		{
			case 1: PORTA = (1 << PA7) | (sevseg%10); break;
			case 2: PORTA = (1 << PA7) | (1 << PA4) | ((sevseg/10)%10); break;
			case 3: PORTA = (1 << PA7) | (1 << PA5) | ((sevseg/100)%10); break;
			case 4: PORTA = (1 << PA7) | (1 << PA4) | (1 << PA5) | ((sevseg/1000)%10); i = 0; break;
		}
		i++;
	}
#endif	/* SEVSEG */



#ifdef MATRIX
/* ================================= MATRIX ================================= */
	uint8 readmatrix(void)
	{
		uint8 m_bill_tomb[12] = {69,14,13,11,22,21,19,38,37,35,70,67};
/*			return values: 		 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, *, #				*/
		
		for(uint8 i = 0; i < 4; i++)
		{
			PORTC &= (0 << PC6) | (0 << PC5) | (0 << PC4) | (0 << PC3);
			PORTC |= (1 << (i + 3));
			
			_delay_ms(1);
			
			for(uint8 j = 0; j < 12; j++)
			{
				if((PINC & 0b01111111)==m_bill_tomb[j])
				{
					return j;
				}
			}
		}
	
		return MATRIX_NO_PRESS;		/* nothing pressed or error */
	}
#endif	/* MATRIX */



#ifdef BUTTONS
/* ================================= BUTTONS ================================= */
	uint8 readbuts(void)
	{
		#if DEBOUNCE == ON
			static uint8 butstate = 1;
			uint8 buts = PING;
			
			if(butstate && buts != 0)
			{
				butstate = 0;
				return buts;
			}
			else
			{
				if(buts == 0)
				{
					butstate = 1;
				}
				return BUTTON_HOLD;
			}
			
		#else
			return PING;
		#endif	/* DEBOUNCE */
	}
#endif	/* BUTTONS */



#ifdef LCD
/* ================================= LCD ================================= */
	void LCD_init(void)
	{
		LCD_DATA_DDR |= 0xF0;			/* data pins set to output */
		LCD_CMD_DDR |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_RW);		/* command pins set to output */
		
		/* rs and rw set to 0 */
		LCD_CMD_PORT &= ~(1 << LCD_RW);
		LCD_CMD_PORT &= ~(1 << LCD_RS);
		
		
		LCD_DATA_PORT = 0x30;			/* function set 4, DL bit = 0 */
		LCD_clock();
		_delay_ms(5);
		
		LCD_DATA_PORT = 0x30;
		LCD_clock();
		_delay_us(100);
		
		LCD_DATA_PORT = 0x30;
		LCD_clock();
		
		LCD_cmd(0x20);
		LCD_cmd(0x28);
		
		LCD_cmd(0x02);
		LCD_cmd(0x01);						/*LCD clear*/
		
		LCD_cmd(0x08 | (LCD_ENABLE<<LCD_E) | (LCD_CURSOR<<LCD_CUR) | (LCD_BLINK<<LCD_BL));
		
		if(LCD_INIT_MESSAGE)
		{
			LCD_text_with_pos("LCD line 1", 1);
			LCD_text_with_pos("LCD line 2", 2);
			LCD_text_with_pos("LCD line 3", 3);
			LCD_text_with_pos("LCD line 4", 4);
			
			if(LCD_INIT_MESSAGE_DELAY)
			_delay_ms(LCD_INIT_MESSAGE_DELAY_TIME);
			LCD_clear();
		}
		
	}
	
	
	void LCD_busy(void)
	{
		
		LCD_DATA_DDR &= ~(1 << LCD_BUSY_PIN);
		LCD_CMD_PORT &= ~(1 << LCD_RS);
		LCD_CMD_PORT |= (1 << LCD_RW);
		
		uint8 busy = 0;
		
		do{
			LCD_CMD_PORT |= (1 << LCD_EN);
			_delay_us(1);
			
			busy = (LCD_DATA_PIN & (1 << LCD_BUSY_PIN));
			
			LCD_CMD_PORT &= ~(1 << LCD_EN);
			_delay_us(1);
			LCD_CMD_PORT |= (1 << LCD_EN);
			_delay_us(1);
			LCD_CMD_PORT &= ~(1 << LCD_EN);
			_delay_us(1);
			
		}while(busy);
		
		LCD_CMD_PORT &= ~(1 << LCD_RW);
		LCD_DATA_DDR |= (1 << LCD_BUSY_PIN);
	}
	
	
	void LCD_clock(void)
	{
		/* Enable bitre: _| |_ */
		LCD_CMD_PORT |= (1 << LCD_EN);
		_delay_us(2);
		LCD_CMD_PORT &= ~(1 << LCD_EN);
		_delay_us(2);
	}
	
	
	void LCD_cmd(uint8 cmd)
	{
		LCD_busy();
		LCD_CMD_PORT &= ~(1 << LCD_RS);
		LCD_CMD_PORT &= ~(1 << LCD_RW);
		LCD_CMD_PORT &= ~(1 << LCD_EN);
		
		LCD_DATA_PORT &= (0x0F);
		LCD_DATA_PORT |= (cmd & 0xF0);
		LCD_clock();
		
		LCD_DATA_PORT &= (0x0F);
		LCD_DATA_PORT |= ((cmd << 4) & 0xF0);
		LCD_clock();
	}
	
	
	void LCD_data(uint8 data)
	{
		LCD_busy();
		LCD_CMD_PORT |= (1 << LCD_RS);
		LCD_CMD_PORT &= ~(1 << LCD_RW);
		LCD_CMD_PORT &= ~(1 << LCD_EN);
		
		LCD_DATA_PORT &= (0x0F);
		LCD_DATA_PORT |= (data & 0xF0);
		LCD_clock();
		
		LCD_DATA_PORT &= (0x0F);
		LCD_DATA_PORT |= ((data << 4) & 0xF0);
		LCD_clock();
	}
	
	
	void LCD_write(char* s)
	{
		while(*s)
		{
			LCD_data(*s);
			s++;
		}
	}
	
	
	void LCD_lineselect(uint8 line)
	{
		/* INPUTS: 1,2,3,4 */
		
		switch(line)
		{
			case 1:
			LCD_cmd(SOR_1);
			break;
			case 2:
			LCD_cmd(SOR_2);
			break;
			case 3:
			LCD_cmd(SOR_3);
			break;
			case 4:
			LCD_cmd(SOR_4);
			break;
		}
	}
	
	
	void LCD_text_with_pos(char* text, uint8 line)
	{
		LCD_lineselect(line);
		LCD_write(text);
	}
	
	
	void LCD_char_with_pos(char text, uint8 sor, uint8 oszlop)
	{
		switch(sor)
		{
			case 1:
			LCD_cmd(SOR_1 + oszlop);
			break;
			case 2:
			LCD_cmd(SOR_2 + oszlop);
			break;
			case 3:
			LCD_cmd(SOR_3 + oszlop);
			break;
			case 4:
			LCD_cmd(SOR_4 + oszlop);
			break;
		}
		
		LCD_data(text);
	}
	
	
	void LCD_clear(void)
	{
		LCD_cmd(0x01);
	}
	
	
	void LCD_rshift(void)
	{
		LCD_cmd(0x1C);
	}
	
	
	void LCD_lshift(void)
	{
		LCD_cmd(0x18);
	}
	
	
	customChar[8]={

		0b11110,
		0b10000,
		0b11100,
		0b10010,
		0b10100,
		0b01010,
		0b11111,
		0b00010
	};


	customChar2[8]={
		0b00000,
		0b01010,
		0b10101,
		0b10001,
		0b01010,
		0b01010,
		0b00100,
		0b00000
	};


	void LCD_creatctg()
	{
		uint8 i=0;
		LCD_cmd (0x40);
		while(i<8) LCD_data(customChar[i++]);
		LCD_cmd(0x80);
		i=0;
		LCD_cmd (0x48);
		while(i<8) LCD_data(customChar2[i++]);
		LCD_cmd(0x80);

	}
#endif	/* LCD */



#ifdef SERIAL
	void ser_init(void)
	{
		/* ================= SERIAL0 ================= */
		UBRR0H = (UBRR >> 8);	/* setup BAUD */
		UBRR0L = UBRR;
		
		UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0) /*| (1 << TXCIE0)*/;		/* enable tx, rx , and rx interrupt*/
		
		UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);		/* 8bit data */
		
		/* ================= SERIAL1 ================= */
		UBRR1H = (UBRR >> 8);
		UBRR1L = UBRR;
		
		UCSR1B = (1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1) /*| (1 << TXCIE1)*/;		/* enable tx, rx , and rx interrupt*/
		
		UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);		/* 8bit data */
	
		if(SERIAL0_INIT_MESSAGE)
		{
			ser0_write_str("Serial0 init OK\r\n");
			if(SERIAL0_INIT_MESSAGE_DELAY)
			_delay_ms(SERIAL0_INIT_MESSAGE_DELAY_TIME);
		}
		
		
		if(SERIAL1_INIT_MESSAGE)
		{
			ser1_write_str("Serial1 init OK\r\n");
			if(SERIAL1_INIT_MESSAGE_DELAY)
			_delay_ms(SERIAL1_INIT_MESSAGE_DELAY_TIME);
		}
	}
	
	
	void ser0_write_char(char betu)
	{
		while(!(UCSR0A & (1 << UDRE0)));
		
		UDR0 = betu;
	}
	
	
	void ser1_write_char(char betu)
	{
		while(!(UCSR1A & (1 << UDRE1)));
		
		UDR1 = betu;
	}
	
	
	void ser0_write_str(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR0A & (1 << UDRE0)));
			
			UDR0 = *str;
			str++;
		}
	}
	
	
	void ser1_write_str(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR1A & (1 << UDRE1)));
			
			UDR1 = *str;
			str++;
		}
	}
	
	
	void ser0_writeline(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR0A & (1 << UDRE0)));
			
			UDR0 = *str;
			str++;
		}
		
		/* write new line and carriage return */
		ser0_write_char(CR_ASCII);
		ser0_write_char(LF_ASCII);
	}
	
	
	void ser1_writeline(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR1A & (1 << UDRE1)));
			
			UDR1 = *str;
			str++;
		}
		
		/* write new line and carriage return */
		ser1_write_char(CR_ASCII);
		ser1_write_char(LF_ASCII);
	}
	
	
/*	only using strings 

	char ser0_read_char(void)
	{
		while(!(UCSR0A & (1 << RXC0)));
		
		return UDR0;
	}
	
	
	char ser1_read_char(void)
	{
		while(!(UCSR1A & (1 << RXC1)));
		
		return UDR1;
	}
*/
#endif	/* SERIAL */



#ifdef MYADC
/* ================================= ADC ================================= */
	void ADC_init(void)
	{
		ADMUX &= (0 << REFS0) | (0 << REFS1);		/* 4.096 V reference voltage */
		
		/* ADC enable, adc interrupt enable, ADC prescaler: 128 */
		ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);		
	}
	
	
	void ADC_start(void)
	{
		ADCSRA |= (1 << ADSC);
	}
#endif	/* ADC */



#if MYSPI == MASTER
/* ================================= SPI MASTER ================================= */
	void SPI_init_master(void)
	{
		/* set MOSI and SCK output, all other input */
		DDR_SPI |= (1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS);
				
		/* enable SPI, Master, set clock rate fck/16, enable interrupt */
		SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0)/* | (1 << SPIE)*/;  /* for the MASTER interrupt is disabled */
	}
	
	
	void SPI_write_char(char betu)
	{
		/* Start transmission */
		SPDR = betu;
		/* Wait for transmission complete */
		while(!(SPSR & (1 << SPIF)));
	}
	
	
	void SPI_write_str(char *str)
	{
		while(*str != '\0')
		{
			SPDR = *str;
			str++;
			
			while(!(SPSR & (1 << SPIF)));
		}
	}
#endif	/* SPI_MASTER */



#if MYSPI == SLAVE
/* ================================= SPI SLAVE ================================= */
	void SPI_init_slave(void)
	{
		DDR_SPI |= (1 << DD_MISO);
				
		/* Enable SPI */
		SPCR |= (1 << SPE) | (1 << SPIE);
	}
	
	
	void SPI_write_char(char betu)
	{
		/* Start transmission */
		SPDR = betu;
		
		/* Wait for transmission complete */
		while(!(SPSR & (1 << SPIF)));
	}
	
	
	void SPI_write_str(char *str)
	{
		while(*str != '\0')
		{
			SPDR = *str;
			str++;
			
			while(!(SPSR & (1 << SPIF)));
		}
	}
#endif	/* SPI_SLAVE */



void rs_init_master(void)
{
	PORTC |= (1 << RS_RE_PIN);
	PORTE |= (1 << RS_DE_PIN);
}

#if RS485 == MASTER
/* ================================= RS 485 MASTER ================================= */

	
	void rs_write_char(char betu)
	{
		while(!(UCSR1A & (1 << UDRE1)));
		
		UDR1 = betu;
	}
	
	void rs_write_str(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR1A & (1 << UDRE1)));
			
			UDR1 = *str;
			str++;
		}
	}
	
	void rs_writeline(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR1A & (1 << UDRE1)));
			
			UDR1 = *str;
			str++;
		}
		
		/* write new line and carriage return */
		ser1_write_char(CR_ASCII);
		ser1_write_char(LF_ASCII);
	}
#endif	/* RS485_MASTER */



void rs_init_slave(void)
{
	PORTC &= (0 << RS_RE_PIN);
	PORTE &= (0 << RS_DE_PIN);
}

#if RS485 == SLAVE
/* ================================= RS 485 SLAVE ================================= */

	
	void rs_write_char(char betu)
	{
		while(!(UCSR1A & (1 << UDRE1)));
		
		UDR1 = betu;
	}
	
	void rs_write_str(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR1A & (1 << UDRE1)));
			
			UDR1 = *str;
			str++;
		}
	}
	
	void rs_writeline(char *str)
	{
		while(*str != '\0')
		{
			while(!(UCSR1A & (1 << UDRE1)));
			
			UDR1 = *str;
			str++;
		}
		
		/* write new line and carriage return */
		ser1_write_char(CR_ASCII);
		ser1_write_char(LF_ASCII);
	}
#endif	/* RS485_SLAVE */



#ifdef TWI
	void TWI_init(void)
	{
		//Nincs eloosztas
		TWSR = 0;
		//TWSR = (0<<TWPS0) | (0<<TWPS1);
		//TWBR >10 - a stabilitashoz
		TWBR = ((F_CPU / SCL_CLK)-16)/2;
		//((8000000/100000)-16)/2=32 > 10 OK - sabil
	}

	//0 ha OK
	//1 ha nem
	uint8 TWI_start(uint8 address)
	{
		//status tarolasara
		uint8 status=0;
		//Start kulsese a master reserol
		//enable + it flag + start
		TWCR=(1<<TWEN) | (1<<TWINT) | (1<<TWSTA);
		//Start befejezesere varakozas
		while(!(TWCR&(1<<TWINT)));
		//Statuszinfo olvasasa
		//TW_STATUS & 0xF8 - 11111000, mert
		//status info TWSR 7-3. biten
		status= TW_STATUS & TW_STATUS_MASK;
		if((status!=TW_START) && (status != TW_REP_START)) return 1;
		//visszaterunk 1-el, mert valami nem OK
		//megadjuk a cimet
		TWDR = address;
		TWCR=(1<<TWEN) | (1<<TWINT);
		while(!(TWCR&(1<<TWINT)));
		status= TW_STATUS & TW_STATUS_MASK;
		if((status!=TW_MR_SLA_ACK) && (status!=TW_MT_SLA_ACK)) return 1;
		return 0;
	}


	void TWI_stop(void)
	{
		//engedélyezés + IT flag + Stop
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
		//várakozás a stop-ra
		while(TWCR & (1<<TWSTO));
	}


	uint8 TWI_rep_start(uint8 address)
	{	//imételt küldése a start-nak
		//0 ha ok, 1 ha nem
		return TWI_start(address);
	}


	uint8 TWI_start_wait(uint8 address)
	{
		//start - csak NACK figyelese
		//ha NACK van, akkor busy - stop kuldese
		//status tarolasara
		uint8 status = 0;
		while ( 1 )
		{
			//Start kulsese a master reserol //enable + it flag + start
			TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
			
			//Start befejezesere varakozas
			while(!(TWCR&(1<<TWINT)));
			// check value of TWI Status Register. Mask prescaler bits.
			status = TW_STATUS;
			if ( (status != TW_START) && (status != TW_REP_START)) continue;
			// cim megadasa
			TWDR = address;										//TWDR -> TWI Data Register
			TWCR = (1<<TWINT) | (1<<TWEN);
			// warakozas
			while(!(TWCR & (1<<TWINT)));
			// statusinfo kiolvasasa
			status = TW_STATUS;
			if ( (status == TW_MT_SLA_NACK )||(status ==TW_MR_DATA_NACK) )
			{
				//foglaltsag figyelese
				TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);				//TWSTO -> TWI STOP Condition
				// start vegrahajtasara var
				while(TWCR & (1<<TWSTO));
				continue;
			}
			break;
		}
		return 0;
	}


	uint8 TWI_write(uint8 data)
	{
		uint8 status=0;
		//adat
		TWDR = data;
		//engedelyezes
		TWCR = (1<<TWEN) | (1<<TWINT);
		//varakozas
		while(!(TWCR&(1<<TWINT)));
		//status
		status=TW_STATUS & TW_STATUS_MASK;
		if(status != TW_MT_DATA_ACK) return 1;
		
		return 0;
	}


	uint8 TWI_read_ACK(void)
	{
		//it twi en + TWEA
		TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
		//varakozas
		while(!(TWCR&(1<<TWINT)));
		//visszateres az adatragiszterrel
		//status= TW_STATUS & TW_STATUS_MASK;
		//if(status!=TW_MR_DATA_ACK) return 0;
		return TWDR;
	}


	uint8 TWI_read_NACK(void)
	{
		//uint8 status=0;
		//it twi en + TWEA
		TWCR = (1<<TWEN) | (1<<TWINT);
		//varakozas
		while(!(TWCR&(1<<TWINT)));
		//visszateres az adatragiszterrel
		//status= TW_STATUS & TW_STATUS_MASK;
		//if(status!=TW_MR_DATA_NACK) return 0;
		return TWDR;
	}


	void RTC_Write(uint8 address, uint8 data_reg, uint8 data)
	{
		//start wait - cim + write
		TWI_start_wait(address|WRITE);
		//write data reg
		TWI_write(data_reg);
		//write data
		TWI_write(data);
		//stop
		TWI_stop();
	}


	//slave cime es adatragiszter cime ahonnan olvasunk
	uint8 RTC_Read(uint8 address, uint8 data_reg)
	{
		uint8 data=0;
		// start wait - cim + write
		TWI_start_wait(address|WRITE);
		//write
		TWI_write(data_reg);
		//start - cim + read
		TWI_start(address|READ);
		//read NACK
		data=TWI_read_NACK();
		//stop
		TWI_stop();
		//return data
		return data;
	}
	
	#ifdef RTC_SET_TIME
		void RTC_write_start_data(void)
		{
			RTC_Write(RTC_ADDR_WRITE, HOURS_ADDR, dec_to_bcd(RTC_HOURS));
			RTC_Write(RTC_ADDR_WRITE, MINS_ADDR, dec_to_bcd(RTC_MINS));
			RTC_Write(RTC_ADDR_WRITE, SECS_ADDR, dec_to_bcd(RTC_SECS));
		}
	#endif /* RTC_SET_TIME */
	
#endif	/* TWI */