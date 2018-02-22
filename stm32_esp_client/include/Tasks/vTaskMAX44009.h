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
#define MAX44009_SAMPLING_PERIOD 		500 		// [ms]
#define MAX44009_CHECK_PERIOD			10			// [samples]

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "xfprintf/xprintf.h"
#include "blue_pill/blue_pill.h"
#include "max44009_m/max44009_m.h"
#include "client_global_sect.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
typedef struct _MAX44009{
	uint32_t 	lux_ambilight_1;
	uint32_t 	lux_ambilight_2;
	uint8_t 	health_1;
	uint8_t 	health_2;
} MAX44009_t;

MAX44009_t MAX44009_Data;
SemaphoreHandle_t xMutex_MAX44009_Data;
uint8_t MAX44009_check_cnt = 0;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vMAX44009_sample(void *pvParameters);
void MAX44009_Reinit(uint8_t sensor_addr, uint8_t *sensor_health);

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
			// Read 1st sensor
			if(MAX44009_Data.health_1 != SENSOR_DEAD){
				MAX44009_Data.lux_ambilight_1 = MAX44009_GetLux(MAX44009_ADDR_1);
				if((MAX44009_Data.lux_ambilight_1 >= MAX44009_LUX_MIN) && (MAX44009_Data.lux_ambilight_1 <= MAX44009_LUX_MAX)){
					station.MAX44009_data.lux_ambilight_1 = MAX44009_Data.lux_ambilight_1;
				}
			}
//			if(MAX44009_Data.health_1 != SENSOR_OK){
//				MAX44009_Reinit(MAX44009_ADDR_1, &MAX44009_Data.health_1);
//			}
			// Read 2nd sensor
			if(MAX44009_Data.health_2 != SENSOR_DEAD){
				MAX44009_Data.lux_ambilight_2 = MAX44009_GetLux(MAX44009_ADDR_2);
				if((MAX44009_Data.lux_ambilight_2 >= MAX44009_LUX_MIN) && (MAX44009_Data.lux_ambilight_2 <= MAX44009_LUX_MAX)){
					station.MAX44009_data.lux_ambilight_2 = MAX44009_Data.lux_ambilight_2;
				}
			}
//			if(MAX44009_Data.health_2 != SENSOR_OK){
//				MAX44009_Reinit(MAX44009_ADDR_2, &MAX44009_Data.health_2);
//			}

//			if(MAX44009_check_cnt >= MAX44009_CHECK_PERIOD){
//				MAX44009_Data.health_1 = SENSOR_CHECK;
//				MAX44009_Data.health_2 = SENSOR_CHECK;
//				MAX44009_check_cnt = 0;
//			}else{
//				MAX44009_check_cnt++;
//			}

			xSemaphoreGive(xMutex_MAX44009_Data);
		}

		vTaskDelayUntil(&pxPreviousWakeTime, MAX44009_SAMPLING_PERIOD);
	}while(1);
	vTaskDelete(NULL);
}

void MAX44009_Reinit(uint8_t sensor_addr, uint8_t *sensor_health){
	if (I2C_IsDeviceConnected(MAX44009_I2C_PORT, sensor_addr)) {
		if(*sensor_health == SENSOR_DEAD){
			MAX44009_Init(sensor_addr);
		}
		*sensor_health = SENSOR_OK;
	}else{
		*sensor_health = SENSOR_DEAD;
	}
}

#endif /* VTASKMAX44009_H_ */
