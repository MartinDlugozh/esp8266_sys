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
#include "blue_pill/blue_pill.h"

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
		if((BMP180_Data.health != SENSOR_DEAD) &&
				(MAX44009_Data.health_1 != SENSOR_DEAD) &&
				(MAX44009_Data.health_2 != SENSOR_DEAD)){
			set_led4(LOW);
		}else{
			set_led4(HIGH);
		}

		vTaskDelay(pxTaskParam->period);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* TASKS_VTASKBLINKER_H_ */
