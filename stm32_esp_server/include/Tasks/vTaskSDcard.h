/*
 * vTaskSDcard.h
 *
 *  Created on: Feb 12, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKSDCARD_H_
#define VTASKSDCARD_H_

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// General STM32 includes
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "common_freertos.h"	// Common convinence functions for FreeRTOS
#include "usart_freertos.h"		// USART convinence functions for FreeRTOS

#include "integer.h"
#include "diskio.h"
#include "fattime.h"
#include "ff.h"

#include "server_global_sect.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define LOG_STR_BUFFER_LEN 			256  		// Always should be less than 512 bytes (memory page size)

// MAVLink FTP macros
#define FILE_NACK 			0
#define FILE_ACK 			1

#define LOG_TYPE_DAILY 		0
#define LOG_TYPE_CONT 		1
#define LOG_TYPE_ALL 		2

#define OP_TYPE_LOAD 		0
#define OP_TYPE_DELETE 		1
#define OP_TYPE_LIST 		2

#define FILE_TRANS_STATE_NOINIT 		0
#define FILE_TRANS_STATE_REQ_LIST 		1
#define FILE_TRANS_STATE_REQ_LIST_RESP 	2
#define FILE_TRANS_STATE_REQ_FILE	 	3
#define FILE_TRANS_STATE_REQ_FILE_RESP 	4

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
typedef struct _SDcardTaskParam {	// blink task parameters
	 uint32_t period; 				// task execution period, ticks (milliseconds)
} SDcardTaskParam_t;

SDcardTaskParam_t sdcardParam;

typedef struct _SD{
	DSTATUS sd_status;
	DRESULT sd_read_result;
	uint8_t sd_drv;
	uint8_t version;
	FATFS fs;
	FIL file;
	FRESULT res;
	DWORD offs;
} SD_t;

uint8_t logging_enabled = 0;		// сделать параметром в EEPROM

SD_t sdcard;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vTaskSDcard(void *pvParameters);
void SDcard_InitStruct(SD_t *sdCardStruct);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void vTaskSDcard(void *pvParameters)
{
	volatile SDcardTaskParam_t *pxTaskParam;
	pxTaskParam = (SDcardTaskParam_t *) pvParameters;
	pxTaskParam->period = 1000;
	TickType_t pxPreviousWakeTime;

	vTaskDelay(2000);
	SDcard_InitStruct(&sdcard);
	sdcard.sd_status = disk_initialize(sdcard.sd_drv);
	f_mount(0, &sdcard.fs);

	pxPreviousWakeTime = xTaskGetTickCount();
	do{

		vTaskSuspend(NULL);

		vTaskDelayUntil(&pxPreviousWakeTime, pxTaskParam->period);
	}while(1);

	f_mount(0, NULL);

	vTaskDelete(NULL);
}

void SDcard_InitStruct(SD_t *sdCardStruct)
{
	sdCardStruct->sd_status = STA_NOINIT;
	sdCardStruct->sd_read_result = 0x03;
	sdCardStruct->sd_drv = 0;
	sdCardStruct->version = 2;
	sdCardStruct->offs = 0;
}

#endif /* VTASKSDCARD_H_ */
