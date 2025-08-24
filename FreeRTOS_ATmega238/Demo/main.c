/*
 * main.c
 *
 *  Created on: 
 *      Author: 
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include "led.h"

#define vLED1FlashTask_PRIORITY			tskIDLE_PRIORITY + 1
#define vLED2FlashTask_PRIORITY			tskIDLE_PRIORITY + 2
#define vLED3FlashTask_PRIORITY			tskIDLE_PRIORITY + 3
#define vLED4FlashTask_PRIORITY			tskIDLE_PRIORITY + 4
#define vLED5FlashTask_PRIORITY			tskIDLE_PRIORITY + 5


// vLED1FlashTask method 
void vLED1FlashTask(void *pvParms)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 100/portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) 
	{
		vLED1Toggle();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

// vLED2FlashTask method
void vLED2FlashTask(void *pvParms)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200/portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		vLED2Toggle();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

// vLED3FlashTask method
void vLED3FlashTask(void *pvParms)
{

	portTickType xLastWakeTime;
	const portTickType xFrequency = 300/portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		vLED3Toggle();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

// vLED4FlashTask method
void vLED4FlashTask(void *pvParms)
{

	portTickType xLastWakeTime;
	const portTickType xFrequency = 400/portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		vLED4Toggle();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

// vLED5FlashTask method
void vLED5FlashTask(void *pvParms)
{

	portTickType xLastWakeTime;
	const portTickType xFrequency = 500/portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		vLED5Toggle();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void init(void);

void vApplicationIdleHook( void );

portSHORT main(void)
{
	init();
	
	xTaskCreate(vLED1FlashTask, (const char *) "vLED1FlashTask", configMINIMAL_STACK_SIZE, NULL, vLED1FlashTask_PRIORITY, NULL);
	xTaskCreate(vLED2FlashTask, (const char *) "vLED2FlashTask", configMINIMAL_STACK_SIZE, NULL, vLED2FlashTask_PRIORITY, NULL);
	xTaskCreate(vLED3FlashTask, (const char *) "vLED3FlashTask", configMINIMAL_STACK_SIZE, NULL, vLED3FlashTask_PRIORITY, NULL);
	xTaskCreate(vLED4FlashTask, (const char *) "vLED4FlashTask", configMINIMAL_STACK_SIZE, NULL, vLED4FlashTask_PRIORITY, NULL);
	xTaskCreate(vLED5FlashTask, (const char *) "vLED5FlashTask", configMINIMAL_STACK_SIZE, NULL, vLED5FlashTask_PRIORITY, NULL);

	vTaskStartScheduler();

	return 0;
}

void init(void)
{
	vLEDInit();
}

void vApplicationIdleHook( void )
{

	// Your custom code here
	//vCoRoutineSchedule();
	
	// Example: Enter low-power mode
    __asm__("sleep");  // AVR sleep instruction

}

