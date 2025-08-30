/*
 * led.c
 *
 *  Created on: 
 *      Author: 
 */

#include <avr/io.h>
#include "led.h"

void vLEDInit(void)
{
	DDRD |= _BV(LED1) | _BV(LED2);
	DDRB |= _BV(LED3) | _BV(LED4) | _BV(LED5);
}

void vLED1Toggle(void)
{
	PORTD ^= _BV(LED1);
}

void vLED2Toggle(void)
{
	PORTD ^= _BV(LED2);
}

void vLED3Toggle(void)
{
	PORTB ^= _BV(LED3);
}

void vLED4Toggle(void)
{
	PORTB ^= _BV(LED4);
}

void vLED5Toggle(void)
{
	PORTB ^= _BV(LED5);
}

