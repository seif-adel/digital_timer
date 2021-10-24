/*
 * 1st_project.c
 *
 *  Created on: Oct 16, 2020
 *      Author: HP
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
unsigned char segments[6]={0,0,0,0,0,0}; /* array which contain the values of the 6 segments*/
void INT0_init(void)
{
    SREG  &= ~(1<<7);              // Disable interrupts by clearing I-bit
    GICR|=(1<<INT0);              /* enable INT0 module */
    MCUCR=(1<<ISC01);             /* work INT0 with falling edge */
    DDRD &= ~(1<<2);                /* make the pin 2 in port d as input */
    PORTD|=(1<<2);               /* activate internal pull up */
    SREG|=(1<<7);                 /* enable I bit of global interrupt*/
}
void INT1_init(void)
{
	SREG  &= ~(1<<7);                          // Disable interrupts by clearing I-bit
	GICR|=(1<<INT1);                           /* enable INT1 module */
	MCUCR= (1<<ISC11)|(1<<ISC10);              /* work INT1 with rising edge */
	DDRD &= ~(1<<3);                           /* make the pin 3 in port d as input */
	SREG|=(1<<7);                              /* enable I bit of global interrupt*/
}
void INT2_init(void)
{
	SREG  &= ~(1<<7);                          // Disable interrupts by clearing I-bit
	GICR|=(1<<INT2);                           /* enable INT2 module */
	MCUCSR&= ~(1<<ISC2);                       /* work INT2 with falling edge */
	DDRB&= ~(1<<2);                            /*make the pin 2 in port B as input  */
	PORTB|=(1<<2);                             /* activate the internal pull up*/
	SREG|=(1<<7);                              /* enable I bit of global interrupt*/
}
ISR(INT0_vect)                                /*to reset the  stopwatch*/
{
	 for(int i=0;i<6;i++)
		 segments[i]=0;
}
ISR(INT1_vect)                                /*to pause the stopwatch*/
{
	TIMSK &= ~ (1<<OCIE1A);
}
ISR(INT2_vect)                                 /* to resume the stopwatch*/
{
	TIMSK|=(1<<OCIE1A);
}
void TIMER1_init(void)
{
	SREG|=(1<<7);
	TCNT1=0;                                                                 /* start count from 0 */
	TCCR1A=(1<<FOC1A);
	TCCR1B=(1<<CS12)|(1<<CS10)|(1<<WGM12);                                   /* make prescalar = 1024 to make 1 tick equal 1ms ,work TIMER1 in compare mode*/
	OCR1A=1000;                                                              /* set the compare value 1000 to make interrupt each 1 second*/
	TIMSK|=(1<<OCIE1A);                                                      /*activate timer1 compare interrupt */
}
ISR(TIMER1_COMPA_vect)
{
	/* increase the right display of seconds one by one and increment minutes and hours with time */
	segments[0]++;

	if(segments[0]==10)
	{
		segments[1]++;
		segments[0]=0;
	}
	if(segments[1]==6)
	{
		segments[2]++;
		segments[1]=0;
	}
	if(segments[2]==10)
	{
		segments[3]++;
		segments[2]=0;
	}
	if(segments[3]==6)
	{
		segments[4]++;
		segments[3]=0;
	}
	if(segments[4]==10)
	{
		segments[5]++;
		segments[4]=0;
	}
}
int main (void)
{
DDRC|=0x0F;                                                                                 /* make first 4 pins in port c as output which will connect to decoder */
DDRA|=0x3F;                                                               /* make first 6 pins in port a output which will connect to npn transistors to comm of 7-segments*/
PORTA=0;                                                                                    /* initialize all transistors as off switch at first*/
PORTC &= 0xF0;
INT0_init();
INT1_init();
INT2_init();
TIMER1_init();
while(1)
{
for(int i=0;i<6;i++)                                                      /* for loop to move on the 6 segments to display all of them */
{
PORTA=(1<<i);                                                             /* this statment for enable each 7 segment and wait 5 ms and then enable the next one*/
PORTC=(PORTC & 0xF0)|(segments[i] & 0x0F);                                /* to put in the current enable 7-segment its current value in the array */
_delay_ms(5);
}
}
}
