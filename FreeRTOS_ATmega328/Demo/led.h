/*
 * led.h
 *
 *  Created on: 
 *      Author: 
 */

#ifndef LED_H_
#define LED_H_

#define LED1 PD6
#define LED2 PD7
#define LED3 PB0
#define LED4 PB1
#define LED5 PB2

void vLEDInit(void);
void vLED1Toggle(void);
void vLED2Toggle(void);
void vLED3Toggle(void);
void vLED4Toggle(void);
void vLED5Toggle(void);

#endif /* LED_H_ */