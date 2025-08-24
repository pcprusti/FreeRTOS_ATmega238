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

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "task.h"
#include <avr/wdt.h>

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the AVR port.
 *----------------------------------------------------------*/

/* Start tasks with interrupts enables. */
#define portFLAGS_INT_ENABLED					( ( StackType_t ) 0x80 )


#if defined( portUSE_WDTO)
//#warning "Watchdog Timer used for scheduler."
#define portSCHEDULER_ISR           WDT_vect

#elif defined( portUSE_TIMER0 )
/* Hardware constants for Timer0. */
//#warning "Timer0 used for scheduler."
#define portSCHEDULER_ISR						TIMER0_COMPA_vect
#define portCLEAR_COUNTER_ON_MATCH				( (uint8_t) _BV(WGM01) )
#define portPRESCALE_64							( (uint8_t) ( _BV( CS01 ) | _BV( CS00 )) )
#define portCLOCK_PRESCALER						( (uint32_t) 64 )
#define portCOMPARE_MATCH_A_INTERRUPT_ENABLE    ( (uint8_t) _BV(OCIE0A) )
#define portOCRL								OCR0A
#define portOCRH								OCR0B
#define portTCCRa								TCCR0A
#define portTCCRb								TCCR0B
#define portTIMSK								TIMSK0
#define portTIFR								TIFR0

#elif defined( portUSE_TIMER1 )
/* Hardware constants for Timer1. */
//#warning "Timer1 used for scheduler."
#define portSCHEDULER_ISR						TIMER1_COMPA_vect
#define portCLEAR_COUNTER_ON_MATCH				( (uint8_t) _BV( WGM12))
#define portPRESCALE_64							( (uint8_t) (_BV(CS11)|_BV(CS10)) )
#define portCLOCK_PRESCALER						( (uint32_t) 64 )
#define portCOMPARE_MATCH_A_INTERRUPT_ENABLE    ( (uint8_t) _BV(OCIE1A) )
#define portOCR									OCR1A
#define portOCRL								OCR1AL
#define portOCRH								OCR1AH
#define portTCCRa								TCCR1A
#define portTCCRb								TCCR1B
#define portTIMSK								TIMSK1
#define portTIFR								TIFR1

#elif defined( portUSE_TIMER2 )
/* Hardware constants for Timer2. */
//#warning "Timer2 used for scheduler."
#define portSCHEDULER_ISR						TIMER2_COMPA_vect
#define portCLEAR_COUNTER_ON_MATCH				( (uint8_t) _BV( WGM21))
#define portPRESCALE_64						    ( (uint8_t) (_BV(CS22)) )
#define portCLOCK_PRESCALER						( (uint32_t) 64 )
#define portCOMPARE_MATCH_A_INTERRUPT_ENABLE    ( (uint8_t) _BV(OCIE2A) )
#define portOCR									OCR2A
#define portTCCRa								TCCR2A
#define portTCCRb								TCCR2B
#define portTIMSK								TIMSK2
#define portTCNT								TCNT2
#define portTIFR								TIFR2

#else
#error "No Timer defined for scheduler."
#endif


/*-----------------------------------------------------------*/

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void TCB_t;
extern volatile TCB_t * volatile pxCurrentTCB;

/*-----------------------------------------------------------*/

/* 
 * Macro to save all the general purpose registers, the save the stack pointer
 * into the TCB.  
 * 
 * The first thing we do is save the flags then disable interrupts.  This is to 
 * guard our stack against having a context switch interrupt after we have already 
 * pushed the registers onto the stack - causing the 32 registers to be on the 
 * stack twice. 
 * 
 * r1 is set to zero as the compiler expects it to be thus, however some
 * of the math routines make use of R1. 
 * 
 * The interrupts will have been disabled during the call to portSAVE_CONTEXT()
 * so we need not worry about reading/writing to the stack pointer. 
 */

#define portSAVE_CONTEXT()									\
	asm volatile (	"push	r0						\n\t"	\
					"in		r0, __SREG__			\n\t"	\
					"cli							\n\t"	\
					"push	r0						\n\t"	\
					"push	r1						\n\t"	\
					"clr	r1						\n\t"	\
					"push	r2						\n\t"	\
					"push	r3						\n\t"	\
					"push	r4						\n\t"	\
					"push	r5						\n\t"	\
					"push	r6						\n\t"	\
					"push	r7						\n\t"	\
					"push	r8						\n\t"	\
					"push	r9						\n\t"	\
					"push	r10						\n\t"	\
					"push	r11						\n\t"	\
					"push	r12						\n\t"	\
					"push	r13						\n\t"	\
					"push	r14						\n\t"	\
					"push	r15						\n\t"	\
					"push	r16						\n\t"	\
					"push	r17						\n\t"	\
					"push	r18						\n\t"	\
					"push	r19						\n\t"	\
					"push	r20						\n\t"	\
					"push	r21						\n\t"	\
					"push	r22						\n\t"	\
					"push	r23						\n\t"	\
					"push	r24						\n\t"	\
					"push	r25						\n\t"	\
					"push	r26						\n\t"	\
					"push	r27						\n\t"	\
					"push	r28						\n\t"	\
					"push	r29						\n\t"	\
					"push	r30						\n\t"	\
					"push	r31						\n\t"	\
					"lds	r26, pxCurrentTCB		\n\t"	\
					"lds	r27, pxCurrentTCB + 1	\n\t"	\
					"in		r0, 0x3d				\n\t"	\
					"st		x+, r0					\n\t"	\
					"in		r0, 0x3e				\n\t"	\
					"st		x+, r0					\n\t"	\
				);

/* 
 * Opposite to portSAVE_CONTEXT().  Interrupts will have been disabled during
 * the context save so we can write to the stack pointer. 
 */

#define portRESTORE_CONTEXT()								\
	asm volatile (	"lds	r26, pxCurrentTCB		\n\t"	\
					"lds	r27, pxCurrentTCB + 1	\n\t"	\
					"ld		r28, x+					\n\t"	\
					"out	__SP_L__, r28			\n\t"	\
					"ld		r29, x+					\n\t"	\
					"out	__SP_H__, r29			\n\t"	\
					"pop	r31						\n\t"	\
					"pop	r30						\n\t"	\
					"pop	r29						\n\t"	\
					"pop	r28						\n\t"	\
					"pop	r27						\n\t"	\
					"pop	r26						\n\t"	\
					"pop	r25						\n\t"	\
					"pop	r24						\n\t"	\
					"pop	r23						\n\t"	\
					"pop	r22						\n\t"	\
					"pop	r21						\n\t"	\
					"pop	r20						\n\t"	\
					"pop	r19						\n\t"	\
					"pop	r18						\n\t"	\
					"pop	r17						\n\t"	\
					"pop	r16						\n\t"	\
					"pop	r15						\n\t"	\
					"pop	r14						\n\t"	\
					"pop	r13						\n\t"	\
					"pop	r12						\n\t"	\
					"pop	r11						\n\t"	\
					"pop	r10						\n\t"	\
					"pop	r9						\n\t"	\
					"pop	r8						\n\t"	\
					"pop	r7						\n\t"	\
					"pop	r6						\n\t"	\
					"pop	r5						\n\t"	\
					"pop	r4						\n\t"	\
					"pop	r3						\n\t"	\
					"pop	r2						\n\t"	\
					"pop	r1						\n\t"	\
					"pop	r0						\n\t"	\
					"out	__SREG__, r0			\n\t"	\
					"pop	r0						\n\t"	\
				);

/*-----------------------------------------------------------*/

/*
 * Perform hardware setup to enable ticks from timer 1, compare match A.
 */
static void prvSetupTimerInterrupt( void );
/*-----------------------------------------------------------*/

/* 
 * See header file for description. 
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
uint16_t usAddress;

	/* Place a few bytes of known values on the bottom of the stack. 
	This is just useful for debugging. */

	*pxTopOfStack = 0x11;
	pxTopOfStack--;
	*pxTopOfStack = 0x22;
	pxTopOfStack--;
	*pxTopOfStack = 0x33;
	pxTopOfStack--;

	/* Simulate how the stack would look after a call to vPortYield() generated by 
	the compiler. */

	/*lint -e950 -e611 -e923 Lint doesn't like this much - but nothing I can do about it. */

	/* The start of the task code will be popped off the stack last, so place
	it on first. */
	usAddress = ( uint16_t ) pxCode;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	/* Next simulate the stack as if after a call to portSAVE_CONTEXT().  
	portSAVE_CONTEXT places the flags on the stack immediately after r0
	to ensure the interrupts get disabled as soon as possible, and so ensuring
	the stack use is minimal should a context switch interrupt occur. */
	*pxTopOfStack = ( StackType_t ) 0x00;	/* R0 */
	pxTopOfStack--;
	*pxTopOfStack = portFLAGS_INT_ENABLED;
	pxTopOfStack--;


	/* Now the remaining registers.   The compiler expects R1 to be 0. */
	*pxTopOfStack = ( StackType_t ) 0x00;	/* R1 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x02;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x03;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x04;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x05;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x06;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x07;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x08;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x09;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x10;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x11;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x13;	/* R13 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x14;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x15;	/* R15 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x16;	/* R16 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x17;	/* R17 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x18;	/* R18 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x19;	/* R19 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x20;	/* R20 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x21;	/* R21 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x22;	/* R22 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x23;	/* R23 */
	pxTopOfStack--;

	/* Place the parameter on the stack in the expected location. */
	usAddress = ( uint16_t ) pvParameters;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	usAddress >>= 8;
	*pxTopOfStack = ( StackType_t ) ( usAddress & ( uint16_t ) 0x00ff );
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0x26;	/* R26 X */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x27;	/* R27 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x28;	/* R28 Y */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x29;	/* R29 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x30;	/* R30 Z */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x031;	/* R31 */
	pxTopOfStack--;

	/*lint +e950 +e611 +e923 */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
	/* Setup the hardware to generate the tick. */
	prvSetupTimerInterrupt();

	/* Restore the context of the first task that is going to run. */
	portRESTORE_CONTEXT();

	/* Simulate a function call end as generated by the compiler.  We will now
	jump to the start of the task the context of which we have just restored. */
	asm volatile ( "ret" );

	/* Should not get here. */
	return pdTRUE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the AVR port will get stopped.  If required simply
	disable the tick interrupt here. */
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch.  The first thing we do is save the registers so we
 * can use a naked attribute.
 */
void vPortYield( void ) __attribute__ ( ( naked ) );
void vPortYield( void )
{
	portSAVE_CONTEXT();
	vTaskSwitchContext();
	portRESTORE_CONTEXT();

	asm volatile ( "ret" );
}
/*-----------------------------------------------------------*/

/*
 * Context switch function used by the tick.  This must be identical to 
 * vPortYield() from the call to vTaskSwitchContext() onwards.  The only
 * difference from vPortYield() is the tick count is incremented as the
 * call comes from the tick ISR.
 */
void vPortYieldFromTick( void ) __attribute__ ( ( naked ) );
void vPortYieldFromTick( void )
{
	portSAVE_CONTEXT();
	if( xTaskIncrementTick() != pdFALSE )
	{
		vTaskSwitchContext();
	}
	portRESTORE_CONTEXT();

	asm volatile ( "ret" );
}
/*-----------------------------------------------------------*/




/**
    Enable the watchdog timer, configuring it for expire after
    (value) timeout (which is a combination of the WDP0
    through WDP3 bits).

    This function is derived from <avr/wdt.h> but enables only
    the interrupt bit (WDIE), rather than the reset bit (WDE).

    Can't find it documented but the WDT, once enabled,
    rolls over and fires a new interrupt each time.

    See also the symbolic constants WDTO_15MS et al.

    Updated to match avr-libc 2.0.0
*/


#if defined( portUSE_WDTO)
static __inline__
__attribute__ ((__always_inline__))
void wdt_interrupt_enable (const uint8_t value)
{
    if (_SFR_IO_REG_P (_WD_CONTROL_REG))
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "out %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "out %0, %2"                "\n\t"
                : /* no outputs */
                : "I" (_SFR_IO_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | (value & 0x07)) )
                : "r0"
        );
    }
    else
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "sts %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "sts %0, %2"                "\n\t"
                : /* no outputs */
                : "n" (_SFR_MEM_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | (value & 0x07)) )
                : "r0"
        );
    }
}
#endif

/*-----------------------------------------------------------*/

/**
    Enable the watchdog timer, configuring it for expire after
    (value) timeout (which is a combination of the WDP0
    through WDP3 bits).

    This function is derived from <avr/wdt.h> but enables both
    the reset bit (WDE), and the interrupt bit (WDIE).

    This will ensure that if the interrupt is not serviced
    before the second timeout, the AVR will reset.

    Servicing the interrupt automatically clears it,
    and ensures the AVR does not reset.

    Can't find it documented but the WDT, once enabled,
    rolls over and fires a new interrupt each time.

    See also the symbolic constants WDTO_15MS et al.

    Updated to match avr-libc 2.0.0
*/

#if defined( portUSE_WDTO)
static __inline__
__attribute__ ((__always_inline__))
void wdt_interrupt_reset_enable (const uint8_t value)
{
    if (_SFR_IO_REG_P (_WD_CONTROL_REG))
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "out %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "out %0, %2"                "\n\t"
                : /* no outputs */
                : "I" (_SFR_IO_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | _BV(WDE) | (value & 0x07)) )
                : "r0"
        );
    }
    else
    {
        __asm__ __volatile__ (
                "in __tmp_reg__,__SREG__"   "\n\t"
                "cli"                       "\n\t"
                "wdr"                       "\n\t"
                "sts %0, %1"                "\n\t"
                "out __SREG__,__tmp_reg__"  "\n\t"
                "sts %0, %2"                "\n\t"
                : /* no outputs */
                : "n" (_SFR_MEM_ADDR(_WD_CONTROL_REG)),
                "r" ((uint8_t)(_BV(_WD_CHANGE_BIT) | _BV(WDE))),
                "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) |
                        _BV(WDIF) | _BV(WDIE) | _BV(WDE) | (value & 0x07)) )
                : "r0"
        );
    }
}
#endif


#if defined(portUSE_WDTO)
/*
 * Setup WDT to generate a tick interrupt.
 */
void prvSetupTimerInterrupt( void )
{
    /* reset watchdog */
    wdt_reset();

    /* set up WDT Interrupt (rather than the WDT Reset). */
    wdt_interrupt_enable( portUSE_WDTO );
}

#elif defined (portUSE_TIMER0)
/*
 * Setup Timer0 compare match A to generate a tick interrupt.
 */
static void prvSetupTimerInterrupt( void )
{
uint32_t ulCompareMatch;
uint8_t ucLowByte;

    /* Using 8bit Timer0 to generate the tick. Correct fuses must be
    selected for the configCPU_CLOCK_HZ clock.*/

    ulCompareMatch = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

    /* We only have 8 bits so have to scale 1024 to get our required tick rate. */
    ulCompareMatch /= portCLOCK_PRESCALER;

    /* Adjust for correct value. */
    ulCompareMatch -= ( uint32_t ) 1;

    /* Setup compare match value for compare match A. Interrupts are disabled
    before this is called so we need not worry here. */
    ucLowByte = ( uint8_t ) ( ulCompareMatch & ( uint32_t ) 0xff );
    portOCR = ucLowByte;

    /* Setup clock source and compare match behavior. */
    portTCCRa = portCLEAR_COUNTER_ON_MATCH;
	portTCCRb = portPRESCALE_64;
	
    /* Enable the interrupt - this is okay as interrupt are currently globally disabled. */
    ucLowByte = portTIMSK;
    ucLowByte |= portCOMPARE_MATCH_A_INTERRUPT_ENABLE;
    portTIMSK = ucLowByte;
}
#elif defined (portUSE_TIMER1)
/*
 * Setup timer 1 compare match A to generate a tick interrupt.
 */
static void prvSetupTimerInterrupt( void )
{
	uint32_t ulCompareMatch;
	uint8_t ucLowByte;

	/* Using 16bit timer 1 to generate the tick.  Correct fuses must be
	selected for the configCPU_CLOCK_HZ clock. */

	ulCompareMatch = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

	/* We only have 16 bits so have to scale to get our required tick rate. */
	ulCompareMatch /= portCLOCK_PRESCALER;

	/* Adjust for correct value. */
	ulCompareMatch -= ( uint32_t ) 1;

	/* Setup compare match value for compare match A.  Interrupts are disabled 
	before this is called so we need not worry here. */
	portOCR = ulCompareMatch;

	/* Setup clock source and compare match behavior. */
	portTCCRa &= ~(_BV(WGM11) | _BV(WGM10));

	ucLowByte = portCLEAR_COUNTER_ON_MATCH | portPRESCALE_64;
	portTCCRb = ucLowByte;

	/* Enable the interrupt - this is okay as interrupt are currently globally
	disabled. */
	ucLowByte = portTIMSK;

	ucLowByte |= portCOMPARE_MATCH_A_INTERRUPT_ENABLE;
	portTIMSK = ucLowByte; 

}

#elif defined (portUSE_TIMER2)
/*
 * Setup Timer2 compare match A to generate a tick interrupt.
 */
static void prvSetupTimerInterrupt( void )
{
uint32_t ulCompareMatch;
uint8_t ucLowByte;

    /* Using 8bit Timer2 to generate the tick. Correct fuses must be
    selected for the configCPU_CLOCK_HZ clock.*/

    ulCompareMatch = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

    /* We only have 8 bits so have to scale 1024 to get our required tick rate. */
    ulCompareMatch /= portCLOCK_PRESCALER;

    /* Adjust for correct value. */
    ulCompareMatch -= ( uint32_t ) 1;

    /* Setup compare match value for compare match A. Interrupts are disabled
    before this is called so we need not worry here. */
    ucLowByte = ( uint8_t ) ( ulCompareMatch & ( uint32_t ) 0xff );
    portOCR = ucLowByte;

    /* Setup clock source and compare match behaviour. */
    portTCCRa |= portCLEAR_COUNTER_ON_MATCH;
	portTCCRb |= portPRESCALE_64;

    /* Enable the interrupt - this is okay as interrupt are currently globally disabled. */
    ucLowByte = portTIMSK;
    ucLowByte |= portCOMPARE_MATCH_A_INTERRUPT_ENABLE;
    portTIMSK = ucLowByte;
}

#else
#error "prvSetupTimerInterrupt is not defined"
#endif
/*-----------------------------------------------------------*/
#if configUSE_PREEMPTION == 1

	/*
	 * Tick ISR for preemptive scheduler.  We can use a naked attribute as
	 * the context is saved at the start of vPortYieldFromTick().  The tick
	 * count is incremented after the context is saved.
	 */
	ISR(portSCHEDULER_ISR, ISR_NAKED) __attribute__ ((hot, flatten));
	
	//void TIMER1_COMPA_vect( void ) __attribute__ ( ( signal, naked ) );
	//void TIMER1_COMPA_vect( void )
	ISR(portSCHEDULER_ISR)
	{
		vPortYieldFromTick();
		asm volatile ( "reti" );
	}
#else

	/*
	 * Tick ISR for the cooperative scheduler.  All this does is increment the
	 * tick count.  We don't need to switch context, this can only be done by
	 * manual calls to taskYIELD();
	 */
	ISR(portSCHEDULER_ISR, ISR_NAKED) __attribute__ ((hot, flatten));
	
	//void TIMER1_COMPA_vect( void ) __attribute__ ( ( signal ) );
	//void TIMER1_COMPA_vect( void )
	ISR(portSCHEDULER_ISR)
	{
		xTaskIncrementTick();
	}
#endif


	
