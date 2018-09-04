/*
 * usart_Xmega.c
 *
 * Created: 29-08-2018 23:55:00
 * Author : AARON JOHN SABU
 */ 

/*#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define baud_rate 9600 */
#define  F_CPU 2000000
//int  prescaller_bit  = ((FCP_U/((baud_rate)*16))-1);
/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA USART interrupt driven driver example source.
 *
 *      This file contains an example application that demonstrates the
 *      interrupt driven USART driver. The code example sends three bytes, waits
 *      for three bytes to be received and tests if the received data equals the
 *      sent data.
 *
 * \par Application note:
 *      AVR1307: Using the XMEGA USART
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * Revision:1694
 * Date:2008?07?2914:21:58+0200(ti,29jul2008)  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "USART.h"
#include "avr_compiler.h"

/*! Number of bytes to send in test example. */
#define NUM_BYTES  8
/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! USART data struct used in example. */
USART_data_t USART_data;
/*! Test data to send. */
char sendArray[NUM_BYTES] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
/*! Array to put received data in. */
char receiveArray[NUM_BYTES];
/*! Success variable, used to test driver. */
bool success;


/*! \brief Example application.
 *
 *  Example application. This example configures USARTC0 for with the parameters:
 *      - 8 bit character size
 *      - No parity
 *      - 1 stop bit
 *      - 9600 Baud
 *
 *  This function then sends three bytes and tests if the received data is
 *  equal to the sent data. The code can be tested by connecting PC3 to PC2. If
 *  the variable 'success' is true at the end of the function, the three bytes
 *  have been successfully sent and received.
*/
int main(void)
{
	/* counter variable. */
	uint8_t i;

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PC3 (TXD0) as output. */
	PORTC.DIRSET   = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR   = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock frequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	/* Enable global interrupts. */
	sei();

	while(true)
	{

	}


	/* Test to see if sent data equals received data. */
	/* Assume success first.*/
/*	success = true;
	for(i = 0; i < NUM_BYTES; i++) {
		 Check that each element is received correctly. 
	if (receiveArray[i] != sendArray[i]) {
			success = false;
		}
	}

	 If success the program ends up inside the if statement.
	if(success){
		while(true);
	}else{
	  	while(true);
	}*/
}








/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
*/
ISR(USARTC0_RXC_vect)
{
	char receiveChar[NUM_BYTES], sendChar[NUM_BYTES];
	USART_RXComplete(&USART_data);
	for(int j = 0; j<NUM_BYTES; j++)
		if (USART_RXBufferData_Available(&USART_data))
			receiveChar[j] = USART_RXBuffer_GetByte(&USART_data);
	for(int j = 0; j<NUM_BYTES; j++)
		sendChar[j] = receiveChar[j];
	for(int j = 0; j<NUM_BYTES; j++)
		UART_TXBuffer_PutByte(&USART_data, sendChar[j]);
}

/*ISR(USARTC0_RXC_vect)                                                                    // before modified
{ 
	USART_RXComplete(&USART_data);
	
}*/

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}



/*void USART_START(uint8_t prescaler){
	
	PORTC_REMAP |= 0x16; //See page 152 in datasheet, remaps the USART0
	
	//PORTC_OUTSET = PIN7_bm; //Let's make PC7 as TX
	PORTC_DIRSET = PIN7_bm; //TX pin as output
 
	//PORTC_OUTSET = PIN6_bm;
	PORTC_DIRCLR = PIN6_bm; //PC6 as RX
	
	USARTC0_BAUDCTRLB = (int)(prescaler>>8); //Just to be sure that BSCALE is 0
	USARTC0_BAUDCTRLA = (int)(prescaler);   
	
	USARTC0_CTRLC = (3<<USART_CHSIZE0_bp)|(1<<USART_SBMODE_bp);
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	
}
void USART_transmit(uint8_t a){
	 
	while( !(USARTC0_STATUS & USART_DREIF_bm ));
	USARTC0_DATA= a;
	}
uint8_t UART_Receive(){
	//USART_transmit(USARTC0_STATUS);
	//USARTC0_STATUS=0x00;
	 while( !(USARTC0_STATUS & USART_RXCIF_bm) ); 
	  uint8_t t  = USARTC0_DATA;
	 return t;
                  }
ISR(USARTC0_RXC_vect){
	
    // USART_transmit('B');
	int RECEIVE = USARTC0_DATA;
	USARTC0_DATA= RECEIVE;
	//USARTE0_CTRLC= USART_RXCIF_bm;
	//_delay_ms(500);
}
int main(void)
{  
	
	
	 PORTF_DIRSET=0x0F;
	 PORTF_DIRCLR=0xF0;
	  sei();
	  USARTC0_CTRLA |= USART_RXCINTLVL_HI_gc;
	  PMIC_CTRL |= PMIC_HILVLEN_bm;	//HILVLEN
	// USARTC0_STATUS= (1<<USART_RXCIF_bp);
	 int data;
	// uint8_t dat=0xFF;
	 USART_START(12);	
	while(1){
			PORTF_OUTSET=PIN7_bm;
			_delay_ms(50);
			PORTF_OUTCLR= PIN7_bm;
			_delay_ms(50);
				//dat='A';
		
		 //data= UART_Receive();
			//USART_transmit(data);
				//USART_transmit('A');
			//	USART_transmit(USARTC0_STATUS);
				//_delay_ms(500);
			
			}
	}*/