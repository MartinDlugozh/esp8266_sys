/*
 * vTaskBMP180Sample.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKBMP180SAMPLE_H_
#define VTASKBMP180SAMPLE_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define BMP180_SAMPLING_PERIOD 		500 		// [ms]

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "blue_pill/blue_pill.h"
#include "bmp180/bmp180.h"
#include "client_global_sect.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
BMP180_t BMP180_Data;
SemaphoreHandle_t xMutex_BMP180_Data;

//uint8_t bmp180_done_flag = 0;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vBMP180_sample(void *pvParameters);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * OS Task: BMP180 sample
 */
void vBMP180_sample(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;
	xMutex_BMP180_Data = xSemaphoreCreateMutex();

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		if(xSemaphoreTake(xMutex_BMP180_Data, portMAX_DELAY) == pdTRUE){
			BMP180_StartTemperature(&BMP180_Data);	/* Start temperature conversion */
			vTaskDelay(BMP180_Data.Delay);			/* Wait delay in microseconds */
			BMP180_ReadTemperature(&BMP180_Data);	/* Read temperature first */

			BMP180_StartPressure(&BMP180_Data, BMP180_Oversampling_UltraHighResolution); /* Start pressure conversion at ultra high resolution */
			vTaskDelay(BMP180_Data.Delay);			/* Wait delay in microseconds */
			BMP180_ReadPressure(&BMP180_Data); 		/* Read pressure value */

			station.BMP180_data.temterature = BMP180_Data.Temperature;
			station.BMP180_data.altitude = BMP180_Data.Altitude;
			station.BMP180_data.pressure = BMP180_Data.Pressure;

			xSemaphoreGive(xMutex_BMP180_Data);
		}

		vTaskDelayUntil(&pxPreviousWakeTime, BMP180_SAMPLING_PERIOD);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKBMP180SAMPLE_H_ */
