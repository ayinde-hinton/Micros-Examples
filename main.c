/*   DESIGNER NAME:  Yin Hinton
   ASSIGNMENT NAME:  Lab 1
         FILE NAME:  Lab 1 main.c
*/
#include "msp.h"
//
// CPET 133 Lab1 - I/O Port Review
//
// author: Jeanne Christman
// original version: 1/6/2022
//
// This is a sample program that toggles an LED on every button push
// - the LED is conncected to port 4, pin 0 (via a 470 ohm resistor)
// - the pushbutton is connected to port 6, pin 0
// - the pushbutton is active low and has a 1k pull up resistor
//
// This program uses a protoboard connected to the TI MSP432 Launchpad Dev Board
//


/**
 * main.c
 */
#include "msp.h"
#include <stdint.h>
void main(/*void*/)
{
    // always include the following line
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	// Configure the port pins as inputs and outputs
	// Use Bit masking
	// ???
	// P4.0 set as output
	P4DIR |= 0b00000001;
	// P6.0 set as input
	P6DIR &= 0b00000000;;
	// turn the LED on - use bitmasking
	;

	while(1) {
	    // zero means active low
	    // P6IN is query bit masked with all zeros an a one on the bit we're concerned with checking
	    if(0 == (P6IN & 0b00000001)) {
	        // determine if button is pushed - active low
	        // toggle LED
	        P4OUT ^= 00000001;
	    }
	    while (0 == (P6IN & 0b00000001));
	    // wait for pin to be released.
	}
	// code should never get here
	return;
}
