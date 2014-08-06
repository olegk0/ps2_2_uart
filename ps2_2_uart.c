/*
 * PS/2 to UART converter/adapter
 * (c) 2014 olegvedi@gmail.com
 * 
 * for Linux (example of use as core keyboard): inputattach --daemon --baud 19200 --ps2serkbd /dev/ttyS0
 * 
 * 
 * based on:
 * Driver for a PS/2 mouse (AVR)
 * (c) Jens Carroll, Inventronik GmbH
 * 
*/
#define __16f628a
#include "pic16f628a.h"

#define uint8_t unsigned char
#define uint16_t unsigned int


__code uint16_t __at (_CONFIG) cfg0 = _FOSC_INTOSCIO & _WDT_OFF & _PWRTE_OFF & _BOREN_OFF & _CP_OFF & _CPD_OFF & _LVP_OFF;

#ifndef KHZ
#define KHZ	4000
#endif

enum
{
	// PS2 clock attached to B0
	CLK_PORT=0,
	// RS232 receive hardwired to B1
	RX_PORT=1,
	// RS232 send hardwired to B2
	TX_PORT=2,
	// PS2 data attached to B3
	DAT_PORT=3,
	// LED attached to B5
	LED_PORT=5,

	CLK_BIT=(1<<CLK_PORT),
	TX_BIT=(1<<TX_PORT),
	RX_BIT=(1<<RX_PORT),
	DAT_BIT=(1<<DAT_PORT),
	LED_BIT=(1<<LED_PORT)
};

#define	BAUD	19200
//9600
#define BAUD_HI	1

#if	(BAUD_HI == 1)
#define	BAUD_FACTOR	(16L*BAUD)
#else
#define	BAUD_FACTOR	(64L*BAUD)
#endif
#define SPBRG_VALUE	(unsigned char)(((KHZ*1000L)-BAUD_FACTOR)/BAUD_FACTOR)

#define BUFFS_SIZE 20

uint8_t usart_buf[BUFFS_SIZE], ps2_buf[BUFFS_SIZE];
uint8_t *ps2_in_ptr, *ps2_out_ptr;
uint8_t ps2_buffcnt;
uint8_t *usart_in_ptr, *usart_out_ptr;
uint8_t usart_buffcnt;

uint8_t ps2_data,bitcount;

static void clear_ps2_buf(void)
{
  ps2_in_ptr = ps2_out_ptr = ps2_buf;
  ps2_buffcnt = 0;

  bitcount = 11;
  ps2_data = 0; 
}

void isr() __interrupt(0)	// call on interrups
{

  if(INTF){	//RB0/INT External Interrupt Flag
      if (bitcount < 11 && bitcount > 2){	// bit 3 to 10 is data. Parity bit, start and stop bits are ignored. 
	ps2_data >>= 1;
	if (PORTB&DAT_BIT)
	  ps2_data |=  0x80;			// store a '1' 
      }
      
      bitcount--;
      if(bitcount == 0){			// all bits received 

	*ps2_in_ptr++ = ps2_data;
	ps2_buffcnt++;

	if(ps2_in_ptr >= ps2_buf + BUFFS_SIZE)
	  ps2_in_ptr = ps2_buf;


	ps2_data = 0;
	bitcount = 11;
    }
    TMR0= 0;
    INTF=0;
  }else if(T0IF){ 	//TMR0 Overflow Interrupt Flag bit
    if(ps2_data)
      clear_ps2_buf();
    T0IF=0;
  }

}

#define t1_on_65ms()	{TMR1H=0;TMR1L=0;TMR1IF=0;TMR1ON=1;}
#define t1_overflow()	(TMR1IF)
#define t1_off()	{TMR1ON=0;TMR1IF=0;}

#define SET_DAT_BIT(data, port, bit) ((data & 0x01) ? (port | bit) : (port & ~bit)) //pulup mode

#define delay_xz_us(X) {for(j = 0; j < X; j++) j=j;}

/*
 * Send one byte to the ps2 device
 *
 * returns TRUE if no timeout occurred and the device responds with ACK
 * otherwise FALSE
 */
uint8_t ps2_send_byte(uint8_t data)
{
    uint8_t j, result = 0, parity = 0;

//    INTE=0;	//disable int interrup
    GIE=1;

	/* clock and data to high by sel input mode*/
    TRISB |= CLK_BIT;
    TRISB |= DAT_BIT;
	
	/* clock now to low */
    TRISB &= ~CLK_BIT;

	/* minimum delay between clock low and data low */
    delay_xz_us(12);//120us

	/* next data to low */
    TRISB  &= ~DAT_BIT;

	/* send start bit (just with this delay) */
    delay_xz_us(2);//20us

	/* release clock as input - hi*/
    TRISB |= CLK_BIT;
    delay_xz_us(5);//50us

    j = 0;
    t1_on_65ms();

    do{
		/* wait until data gets low (ack from device) */
	while ((PORTB & CLK_BIT) && !t1_overflow());
		/* timer2 overflow? */
	if (t1_overflow()) break;

	if(j<8) {
	    TRISB = SET_DAT_BIT(data, TRISB, DAT_BIT);
	    if (data & 0x01)
		parity ^= 0x01;

	    data >>= 1;
	} else if(j==8){
			/* insert parity */
	    TRISB = SET_DAT_BIT(~parity, TRISB, DAT_BIT);

	} else if(j>8){
			/* clock and data as inputs again */
	    TRISB |= DAT_BIT;
	    TRISB |= CLK_BIT;

	    if (j==10) {	
		/* receive ACK eventually   wait until data gets low (ack from device) */
		while ((PORTB & DAT_BIT) && !t1_overflow());
		if (!t1_overflow())
			result = 1;

		while ((PORTB & DAT_BIT) && (PORTB & CLK_BIT) && !t1_overflow());
		if (t1_overflow())
			result = 0;
		break;
	    }
	}
		
		/* wait until clock gets high or timeout */
	while ((!(PORTB & CLK_BIT)) && !t1_overflow());
	if (t1_overflow())
	    break;
	j++;
    } while (j<11);

	/* clock and data as input */
    TRISB |= DAT_BIT;
    TRISB |= CLK_BIT;

	/* stop timer */
    t1_off();
    
    clear_ps2_buf();
	/* clear interrupt flag bit (write a 1) to prevent ISR entry upon irpt enable */
    INTF=0;
//    INTE=1;	//enable int interrups
    GIE=1;
    return result;
}

void main(void)
{

//	setup port
  NOT_RBPU=0;	// Enable pullups

  TRISB=RX_BIT|CLK_BIT|DAT_BIT;		// Setup I/O on port B    1 - inp
  PORTB=0;

//	setup USART
  SPBRG=SPBRG_VALUE;	// Baud Rate register, calculated by macro
  BRGH=BAUD_HI;

  SYNC=0;			// Disable Synchronous/Enable Asynchronous
  SPEN=1;			// Enable serial port
  TXEN=1;			// Enable transmission mode
  CREN=1;			//enable rcv

//	setup timer0
  T0CS=0;		// Use internal clock for TMR0
  PSA=1;		//no divider
//  PS2 = 1;
//  PS1 = 1;
//  PS0 = 1;
  TMR0= 0;		// Set TMR0 to overflow after 256 clocks - 1M/256 = 0,000256 c

//	setup timer1
  TMR1CS=0;		//int freq
  TMR1ON=0;		//off
  T1CKPS0=0;		//divider 1:1
  T1CKPS1=0;		//divider 1:1
  
//	setup ints
  INTCON=0x00;			// Clear interrupt register completely

//  PEIE=1;			//Peripheral Interrupt Enable bit
  T0IE=1;			// TMR0 Overflow Interrupt Enable bit
  INTE=1;			// RB0/INT External Interrupt Enable bit

  PIE1=0;			//PERIPHERAL INTERRUPT ENABLE REGISTER
//	RCIE=1;			//USART Receive Interrupt Enable bit
//	TXIE=1;			//USART Transmit Interrupt Enable bit
//  TMR1IE=1;			//tmr1 owf
  PIR1=0;			//PERIPHERAL INTERRUPT FLAG REGISTER

  INTEDG=0;		//int0 Interrupt on falling
	
  usart_in_ptr = usart_out_ptr = usart_buf;
  usart_buffcnt = RCREG;//read for clear rcv flag
  usart_buffcnt = 0;
  
  clear_ps2_buf();

  GIE=1;	//global interrupt
	
  while(1)
  {
    if(RCIF){	//USART Receive Interrupt Flag
      *usart_in_ptr++ = RCREG;
      usart_buffcnt++;
      // pointer wrapping

      if(usart_in_ptr >= usart_buf + BUFFS_SIZE)
	usart_in_ptr = usart_buf;
//	  RCIF=0;
    }

    if(usart_buffcnt || ps2_buffcnt)	//for monitoring buffers
      PORTB = LED_BIT;	// LED on.
    else
      PORTB &= ~LED_BIT;	// LED off.

    if(TXIF){	//USART Transmit Interrupt Flag
      if(ps2_buffcnt){ 	// data in buf
	TXREG = *ps2_out_ptr++;	// put byte to tx reg
	if(ps2_out_ptr >= ps2_buf + BUFFS_SIZE) // pointer wrapping
	  ps2_out_ptr = ps2_buf;

	ps2_buffcnt--;	// decrement buffer count 

      }
//	  TXIF=0;
    }

    if(usart_buffcnt){	//to ps2
      ps2_send_byte(*usart_out_ptr++);	// put byte to ps2 line
      if(usart_out_ptr >= usart_buf + BUFFS_SIZE) // pointer wrapping
	usart_out_ptr = usart_buf;

      usart_buffcnt--;	// decrement buffer count 

    }

  }
  
}
