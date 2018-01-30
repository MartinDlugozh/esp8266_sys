/*
 * vTaskMAX44009.h
 *
 *  Created on: Jan 30, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKMAX44009_H_
#define VTASKMAX44009_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define MAX44009_SAMPLING_PERIOD 		200 		// [ms]

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "blue_pill/blue_pill.h"
#include "max44009_m/max44009_m.h"
#include "client_global_sect.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
typedef struct _MAX44009{
	uint32_t 	lux_ambilight_1;
	uint32_t 	lux_ambilight_2;
} MAX44009_t;

MAX44009_t MAX44009_Data;
SemaphoreHandle_t xMutex_MAX44009_Data;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vMAX44009_sample(void *pvParameters);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * OS Task: MAX44009 sample
 */
void vMAX44009_sample(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;
	xMutex_MAX44009_Data = xSemaphoreCreateMutex();

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		if(xSemaphoreTake(xMutex_MAX44009_Data, portMAX_DELAY) == pdTRUE){

			MAX44009_Data.lux_ambilight_1 = MAX44009_GetLux(MAX44009_ADDR_1);
			MAX44009_Data.lux_ambilight_2 = MAX44009_GetLux(MAX44009_ADDR_2);

			station.MAX44009_data.lux_ambilight_1 = MAX44009_Data.lux_ambilight_1;
			station.MAX44009_data.lux_ambilight_2 = MAX44009_Data.lux_ambilight_2;

			xSemaphoreGive(xMutex_MAX44009_Data);
		}

		vTaskDelayUntil(&pxPreviousWakeTime, MAX44009_SAMPLING_PERIOD);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKMAX44009_H_ */
