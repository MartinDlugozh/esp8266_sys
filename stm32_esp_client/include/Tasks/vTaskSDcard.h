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
#define LOG_STR_BUFFER_LEN 			256  		// Always should be less than 512 bytes (memory page size)

#define EEPROM_SIGNATURE 			128
#define EEPROM_ADDR_LOG_NUMBER 		VirtAddVarTab[1]
#define EEPROM_ADDR_EEP_SIGNATURE 	VirtAddVarTab[10]

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
#define FILE_TRANS_STATE_SEN_LIST 		5
#define FILE_TRANS_STATE_SEN_FILS 		6

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
	FILINFO fileinfo;
	DIR dir;
	FRESULT res;
	DWORD offs;
	char file_name[16];
} SD_t;

uint8_t logging_enabled = 0;		// сделать параметром в EEPROM

SD_t sdcard;

TaskHandle_t xTaskHandleSDcard;

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
//	TickType_t pxPreviousWakeTime;

	vTaskDelay(2000);
	SDcard_InitStruct(&sdcard);
	sdcard.sd_status = disk_initialize(sdcard.sd_drv);
	f_mount(0, &sdcard.fs);
	DEBUG_printf("SD init. Status: %d\r\n", sdcard.sd_status);

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
	// We need it because of no RTC (or some another time source) present
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
	DEBUG_printf("SD log number: %d\r\n", log_number);

	// Create new log file
	sprintf(sdcard.file_name, "test_%d.log", log_number);	// Bufferize log name
	sdcard.res = f_open(&sdcard.file, sdcard.file_name, FA_WRITE | FA_OPEN_ALWAYS);  // Create file

	f_sync(&sdcard.file);		// Write sector
	f_close(&sdcard.file);		// Close file

	DEBUG_printf("SD log create: %d\r\n", sdcard.res);

	// Разрешаем запись лога; если принят запрос на передачу файла, то запись необходимо приостанавливать
	logging_enabled = 1;
	DEBUG_printf("SD logging_enabled: %d\r\n", logging_enabled);

//	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		if(logging_enabled){
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

			// Bufferize ans send collected data to SD-card
			// Please, use only global data declared in client_global_sect.h (it may be more portable and independent)
			char *sd_str_p = sd_str;
			memset(sd_str, 0, LOG_STR_BUFFER_LEN);
			sd_str_p += sprintf(sd_str_p, "%lu, ", millis());
			sd_str_p = ftoa(station.SHT11_data.temperature, sd_str_p, 3);
			sd_str_p += sprintf(sd_str_p, ", ");
			sd_str_p = ftoa(station.SHT11_data.humidity, sd_str_p, 3);
			sd_str_p += sprintf(sd_str_p, ", ");
			sd_str_p = ftoa(station.BMP180_data.temperature, sd_str_p, 3);
			sd_str_p += sprintf(sd_str_p, ", %lu, ", station.BMP180_data.pressure);
			sd_str_p += sprintf(sd_str_p, "%lu, ", station.MAX44009_data.lux_ambilight_1);
			sd_str_p += sprintf(sd_str_p, "%lu\n", station.MAX44009_data.lux_ambilight_2);

			sdcard.res = f_write(&sdcard.file, sd_str, strlen(sd_str), &str_len);
			if(sdcard.res){
			}else{
				sdcard.offs += str_len;
			}

			f_sync(&sdcard.file);		// Write sector
			f_close(&sdcard.file);		// Close file
		}

//		vTaskDelayUntil(&pxPreviousWakeTime, pxTaskParam->period);
		vTaskDelay(pxTaskParam->period);
		uxHighWaterMark[HIGH_WATERMARK_SD_CARD] = uxTaskGetStackHighWaterMark(NULL);
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
