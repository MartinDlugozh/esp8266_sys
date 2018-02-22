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
#include "UART_freertos.h"		// USART convinence functions for FreeRTOS
#include "xprintf.h"

#include "integer.h"
#include "diskio.h"
#include "fattime.h"
#include "ff.h"

#include "eeprom.h"

#include "client_global_sect.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define LOG_STR_BUFFER_LEN 	256

#define EEPROM_SIGNATURE 			128
#define EEPROM_ADDR_LOG_NUMBER 		VirtAddVarTab[1]
#define EEPROM_ADDR_EEP_SIGNATURE 	VirtAddVarTab[10]

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
	char file_name[16];
} SD_t;

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

	char sd_str[LOG_STR_BUFFER_LEN];
	UINT str_len = 0;

	// Check EEPROM signature
	uint16_t flash_status = 0;
	uint16_t eep_temp = 0;
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);		/* Enable Prefetch Buffer */
    FLASH_SetLatency(FLASH_Latency_2);   						/* Flash 2 wait state */
    FLASH_Unlock();												// Unlock the Flash Program Erase controller
	flash_status = EE_ReadVariable(EEPROM_ADDR_EEP_SIGNATURE, &eep_temp);
	if((flash_status != 0) || (eep_temp != EEPROM_SIGNATURE)){
		flash_status = EE_Format();
		flash_status = EE_WriteVariable(EEPROM_ADDR_EEP_SIGNATURE, EEPROM_SIGNATURE);
	}

	// Get log number from EEPROM
	uint16_t log_number = 0;
	flash_status = EE_ReadVariable(EEPROM_ADDR_LOG_NUMBER, &log_number);
	if(flash_status != NO_VALID_PAGE){
		if(log_number < UINT16_MAX){
			log_number++;
		}else{
			log_number = 0;
		}
		flash_status = EE_WriteVariable(EEPROM_ADDR_LOG_NUMBER, log_number);
	}
	FLASH_Lock();

	// Create new log file
	sprintf(sdcard.file_name, "test_%d.log", log_number);
	sdcard.res = f_open(&sdcard.file, sdcard.file_name, FA_WRITE | FA_OPEN_ALWAYS); // FA_CREATE_ALWAYS

	f_sync(&sdcard.file);
	f_close(&sdcard.file);

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
//		uint32_t time = xTaskGetTickCount();

		sdcard.res = f_open(&sdcard.file, sdcard.file_name, FA_WRITE | FA_OPEN_ALWAYS);
		f_lseek(&sdcard.file, sdcard.offs);

		if(sdcard.offs < 10){
			sprintf(sd_str, "Log created: %lu\n", millis());
			sdcard.res = f_write(&sdcard.file, sd_str, strlen(sd_str), &str_len);
			if(sdcard.res){
			}else{
				sdcard.offs += str_len;
			}
		}

		////
		// Bufferize ans send collected data to SD-card
		char *sd_str_p = sd_str;
		memset(sd_str, 0, LOG_STR_BUFFER_LEN);
		sd_str_p += sprintf(sd_str_p, "%lu, ", millis());
		sd_str_p = ftoa(station.SHT11_data.temperature, sd_str_p, 3);
		sd_str_p += sprintf(sd_str_p, ", ");
		sd_str_p = ftoa(station.SHT11_data.humidity, sd_str_p, 3);
		sd_str_p += sprintf(sd_str_p, ", ");
		sd_str_p = ftoa(station.BMP180_data.temterature, sd_str_p, 3);
		sd_str_p += sprintf(sd_str_p, ", %lu, ", station.BMP180_data.pressure);
		sd_str_p += sprintf(sd_str_p, "%lu, ", station.MAX44009_data.lux_ambilight_1);
		sd_str_p += sprintf(sd_str_p, "%lu\n", station.MAX44009_data.lux_ambilight_2);

		sdcard.res = f_write(&sdcard.file, sd_str, strlen(sd_str), &str_len);
		if(sdcard.res){
		}else{
			sdcard.offs += str_len;
		}

		f_sync(&sdcard.file);
		f_close(&sdcard.file);

		vTaskDelayUntil(&pxPreviousWakeTime, pxTaskParam->period);
//		vTaskDelay(pxTaskParam->period);
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
