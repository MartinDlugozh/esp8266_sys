/*
 * vTaskBlinker.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 */

#ifndef TASKS_VTASKBLINKER_H_
#define TASKS_VTASKBLINKER_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "blue_pill.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
typedef struct BlinkTaskParam_t {	// blink task parameters
	 uint32_t period; 				// task execution period, ticks (milliseconds)
} BlinkTaskParam;

BlinkTaskParam blinkParam;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vBlinker(void *pvParameters);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * OS Task: Blinker
 *
 * Onboard LED blinking (PC13)
 * Uses onboard_led_toggle() method
 */
void vBlinker(void *pvParameters)
{
	volatile BlinkTaskParam *pxTaskParam;
	pxTaskParam = (BlinkTaskParam *) pvParameters;

	do{
		onboard_led_toggle();
		vTaskDelay(pxTaskParam->period);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* TASKS_VTASKBLINKER_H_ */
