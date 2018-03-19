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
#define BMP180_CHECK_PERIOD			10			// [samples]

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
#include "bmp180/bmp180.h"
#include "client_global_sect.h"
#include "Tasks/vTaskDebug.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
BMP180_t BMP180_Data;
SemaphoreHandle_t xMutex_BMP180_Data;
uint8_t BMP180_check_cnt = 0;

TaskHandle_t xTaskHandleBMP180Sample;

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
			if(BMP180_Data.health != SENSOR_DEAD){

				BMP180_StartTemperature(&BMP180_Data);	/* Start temperature conversion */
				vTaskDelay(BMP180_Data.Delay);			/* Wait delay in microseconds */
				BMP180_ReadTemperature(&BMP180_Data);	/* Read temperature first */

				BMP180_StartPressure(&BMP180_Data, BMP180_Oversampling_UltraHighResolution); /* Start pressure conversion at ultra high resolution */
				vTaskDelay(BMP180_Data.Delay);			/* Wait delay in microseconds */
				BMP180_ReadPressure(&BMP180_Data); 		/* Read pressure value */

				if((BMP180_Data.Temperature >= BMP180_TEMPERATURE_MIN) && (BMP180_Data.Temperature <= BMP180_TEMPERATURE_MAX)){
					station.BMP180_data.temperature = BMP180_Data.Temperature;
				}
				if((BMP180_Data.Altitude >= BMP180_ALTITUDE_MIN) && (BMP180_Data.Altitude <= BMP180_ALTITUDE_MAX)){
					station.BMP180_data.altitude = BMP180_Data.Altitude;
				}
				if((BMP180_Data.Pressure >= BMP180_PRESSURE_MIN) && (BMP180_Data.Pressure <= BMP180_PRESSURE_MAX)){
					station.BMP180_data.pressure = BMP180_Data.Pressure;
				}
			}
//			if(BMP180_Data.health != SENSOR_OK){
//				if (I2C_IsDeviceConnected(BMP180_I2C, BMP180_I2C_ADDRESS)) {
//					if(BMP180_Data.health == SENSOR_DEAD){
//						if (BMP180_Init() == BMP180_Result_Ok) {		// Try to reinitialize BMP180 sensor
//							BMP180_GetPressureAtSeaLevel(101325, 0);
//							BMP180_Data.health = SENSOR_OK;
//						}
//					}
//				}else{
//					BMP180_Data.health = SENSOR_DEAD;
//				}
//			}
//
//			if(BMP180_check_cnt >= BMP180_CHECK_PERIOD){
//				BMP180_Data.health = SENSOR_CHECK;
//				BMP180_check_cnt = 0;
//			}else{
//				BMP180_check_cnt++;
//			}

			xSemaphoreGive(xMutex_BMP180_Data);
		}

		vTaskDelayUntil(&pxPreviousWakeTime, BMP180_SAMPLING_PERIOD);
		uxHighWaterMark[HIGH_WATERMARK_BMP180S] = uxTaskGetStackHighWaterMark(NULL);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKBMP180SAMPLE_H_ */
