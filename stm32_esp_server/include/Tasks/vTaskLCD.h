/*
 * vTaskLCD.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKLCD_H_
#define VTASKLCD_H_
/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define NUM_VIEWS_MAX 3

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "ssd1306.h"		// SSD1306 I2C OLED display driver
#include "LiquidCrystal_I2C.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
typedef struct LCDUpTaskParam_t {
	float temp;
	uint32_t press;
	float alt;
	uint32_t lux_ambilight_1;
	uint32_t lux_ambilight_2;
	uint8_t view;
	uint32_t period; /* период, миллисекунды*/
} LCDUpTaskParam;

LCDUpTaskParam lcdParam;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * OS Task: LCD Updater
 */
void vLCD_Update(void *pvParameters)
{
	volatile LCDUpTaskParam *pxTaskParam;
	pxTaskParam = (LCDUpTaskParam *) pvParameters;
	TickType_t pxPreviousWakeTime;
	char lcd_buff[16];
	uint8_t prev_screen = 0;

	// Init 16x2 LCD
	LCDI2C_init(0x3F, 16, 2);
	// ------- Quick 3 blinks of backlight  -------------
	for(uint8_t i = 0; i < 3; i++)
	{
		LCDI2C_backlight();
		vTaskDelay(250);
		LCDI2C_noBacklight();
		vTaskDelay(250);
	}
	LCDI2C_backlight(); // finish with backlight on
	//-------- Write characters on the display ------------------

	// NOTE: Cursor Position: (CHAR, LINE) start at 0
	LCDI2C_clear();

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		if(prev_screen != pxTaskParam->view){
			prev_screen = pxTaskParam->view;
			LCDI2C_clear();
		}else{
			LCDI2C_clear_fast(lcd_buff);
		}


		switch(pxTaskParam->view)
		{
		case 0:
		{
			// Display time since boot [ms]
			memset(lcd_buff, '\0', 16);
			sprintf(lcd_buff, "MS: %lu", millis());
			LCDI2C_setCursor(0, 0);
			LCDI2C_write_String(lcd_buff);

			break;
		}
		case 1:
		{
			// Display temprature [degC]
			memset(lcd_buff, '\0', 16);
			sprintf(lcd_buff, "T: ");
			LCDI2C_setCursor(0, 0);
			LCDI2C_write_String(lcd_buff);

			memset(lcd_buff, '\0', 16);
			LCDI2C_setCursor(3, 0);
			ftoa(pxTaskParam->temp, lcd_buff, 1);
			LCDI2C_write_String(lcd_buff);

			// Display pressure [pa]
			memset(lcd_buff, '\0', 16);
			LCDI2C_setCursor(0, 1);
			sprintf(lcd_buff, "P %lu", pxTaskParam->press);
			LCDI2C_write_String(lcd_buff);
			break;
		}
		case 2:
		{
			// Display 1st ambient light [lx]
			memset(lcd_buff, '\0', 16);
			sprintf(lcd_buff, "L1: %lu", pxTaskParam->lux_ambilight_1);
			LCDI2C_setCursor(0, 0);
			LCDI2C_write_String(lcd_buff);

			// Display 2nd ambient light [lx]
			memset(lcd_buff, '\0', 16);
			sprintf(lcd_buff, "L2: %lu", pxTaskParam->lux_ambilight_2);
			LCDI2C_setCursor(0, 1);
			LCDI2C_write_String(lcd_buff);
			break;
		}
		default:
			break;
		}

		vTaskDelayUntil(&pxPreviousWakeTime, pxTaskParam->period);
	}while(1);
	vTaskDelete(NULL);
}


#endif /* VTASKLCD_H_ */
