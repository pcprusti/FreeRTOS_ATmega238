/*
 * FreeRTOS Kernel V10.4.6
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
*/

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

#include <avr/wdt.h>

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		int
#define portSTACK_TYPE	uint8_t
#define portBASE_TYPE	char

typedef portSTACK_TYPE StackType_t;
typedef signed char BaseType_t;
typedef unsigned char UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif
/*-----------------------------------------------------------*/

/* Critical section management. */
#define portENTER_CRITICAL()		asm volatile ( "in		__tmp_reg__, __SREG__" :: );	\
									asm volatile ( "cli" :: );								\
									asm volatile ( "push	__tmp_reg__" :: )

#define portEXIT_CRITICAL()			asm volatile ( "pop		__tmp_reg__" :: );				\
									asm volatile ( "out		__SREG__, __tmp_reg__" :: )

#define portDISABLE_INTERRUPTS()	asm volatile ( "cli" :: );
#define portENABLE_INTERRUPTS()		asm volatile ( "sei" :: );
/*-----------------------------------------------------------*/


#if defined(WDIE) && defined(WDIF)       /* If Enhanced WDT with interrupt capability is available */
#ifndef portUSE_WDTO
//#define portUSE_WDTO        WDTO_15MS    // portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick
#endif //portUSE_WDTO
											/* Watchdog period options:     WDTO_15MS
											                                WDTO_30MS
											                                WDTO_60MS
											                                WDTO_120MS
											                                WDTO_250MS
											                                WDTO_500MS
											                                WDTO_1S
											                                WDTO_2S
											*/
#if(defined(portUSE_WDTO) && (portUSE_WDTO == WDTO_15MS))
#define portTICK_PERIOD_MS 15
#endif //((defined #ifdef portUSE_WDTO) && (portUSE_WDTO == WDTO_15MS))
#endif //defined(WDIE) && defined(WDIF)

#ifndef portUSE_TIMER0
//#define portUSE_TIMER0       // portUSE_TIMERx to use the TIMERx for xTaskIncrementTick
#endif

#ifndef portUSE_TIMER1
//#define portUSE_TIMER1       // portUSE_TIMERx to use the TIMERx for xTaskIncrementTick
#endif

#ifndef portUSE_TIMER2
#define portUSE_TIMER2       // portUSE_TIMERx to use the TIMERx for xTaskIncrementTick
#endif



/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#ifndef portTICK_PERIOD_MS
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#endif
#define portBYTE_ALIGNMENT			1
#define portNOP()					asm volatile ( "nop" );
/*-----------------------------------------------------------*/

/* Kernel utilities. */
extern void vPortYield( void ) __attribute__ ( ( naked ) );
#define portYIELD()					vPortYield()
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

