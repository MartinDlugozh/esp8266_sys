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
#define NUM_VIEWS_MAX 4

#define LCD_VIEW_MAIN 			0
#define LCD_VIEW_SETTINGS		1
#define LCD_VIEW_LOAD_DATA		2
#define LCD_VIEW_VIEW_DATA		3
#define LCD_VIEW_SYNC_TIME		4

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "ssd1306.h"		// SSD1306 I2C OLED display driver
#include "LiquidCrystal_I2C.h"
#include "Tasks/vTaskSDcard.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
typedef struct LCDUpTaskParam_t {
	float temp;
	uint32_t press;
	float alt;
	uint32_t lux_ambilight_1;
	uint32_t lux_ambilight_2;
	float sht_temp;
	float humidity;
	float dewpoint;
	uint8_t view;
	uint32_t period; /* период, миллисекунды*/
} LCDUpTaskParam;

LCDUpTaskParam lcdParam;

typedef struct _view_controls{
	uint8_t ok;
	uint8_t plus;
	uint8_t minus;
	uint8_t back;
} view_controls_t;

view_controls_t lcd_view_controls;

uint8_t view = 0;
uint8_t option[5];

uint32_t time = 0;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void viewMain(view_controls_t *view_controls)
{
	view_controls_t control;
	control = *view_controls;
	char view_buffer[16];
	size_t prevlen;

	switch(option[view]){
	case 0: // Main screen
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view] = 4;
		}else if(control.back){

		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "MS: %lu", time);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "STA: %d", client_station[0].HEARTBEAT_data.system_id);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);

		break;
	}
	case 1:	// Settings
	{
		if(control.ok){
			view = LCD_VIEW_SETTINGS;
		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "    SETTINGS");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "   ");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 2:	// Load data
	{
		if(control.ok){
			view = LCD_VIEW_LOAD_DATA;
			option[view] = 0;
		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      LOAD");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      DATA");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 3:	// View data
	{
		if(control.ok){
			view = LCD_VIEW_VIEW_DATA;
		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      VIEW");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      DATA");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 4:	// Sync time
	{
		if(control.ok){
			view = LCD_VIEW_SYNC_TIME;
		}else if(control.plus){
			option[view] = 0;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "   SYNCRONIZE");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      TIME");
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	default:
		break;
	}

	control.ok = 0;
	control.plus = 0;
	control.minus = 0;
	control.back = 0;
	*view_controls = control;
}

void viewLoadData(view_controls_t *view_controls)
{
	view_controls_t control;
	control = *view_controls;
	char view_buffer[16];
	size_t prevlen;

	switch(option[view]){
	case 0: // Load logs
	{
		if(control.ok){
			if(client_station[0].HEARTBEAT_data.system_id != 0){
				client_station[0].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_LIST;
				option[view] = 4;
			}else{

			}
		}else if(control.plus){
			option[view] = 1;
		}else if(control.minus){
			option[view] = 1;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Load logs       ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      OK - BACK ");
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 1:	// Delete logs
	{
		if(control.ok){
			view = LCD_VIEW_MAIN;
		}else if(control.plus){
			option[view] = 0;
		}else if(control.minus){
			option[view] = 0;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Delete logs     ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      OK - BACK ");
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 2:	// Are you sure?
	{
		if(control.ok){

		}else if(control.plus){

		}else if(control.minus){

		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Are you sure?   ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      OK - BACK ");
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 3:	// Deleting
	{
		if(control.ok){

		}else if(control.plus){

		}else if(control.minus){

		}else if(control.back){

		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "    Deleting    ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%d of %d",
				client_station[0].log_file_op.log_file_cnt,
				client_station[0].log_file_op.log_file_seq);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 4:	// Loading
	{
		if(control.ok){

		}else if(control.plus){

		}else if(control.minus){

		}else if(control.back){

		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "    Loading     ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%d of %d",
				client_station[0].log_file_op.log_file_cnt,
				client_station[0].log_file_op.log_file_seq);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 5:	// Done
	{
		if(control.ok){
			option[view] = 0;
		}else if(control.plus){
			option[view] = 0;
		}else if(control.minus){
			option[view] = 0;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "      DONE =)   ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "                ");
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	default:
		break;
	}
}

void viewViewData(view_controls_t *view_controls)
{
	view_controls_t control;
	control = *view_controls;
	char view_buffer[16];
	size_t prevlen;

	switch(option[view]){
	case 0: // Temperature BMP180
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view] = 6;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Temperature degC");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[0].BMP180_data.temperature, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);

		break;
	}
	case 1:	// Pressure BMP180
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Pressure      pa");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%lu", client_station[0].BMP180_data.pressure);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 2:	// Temperature SHT11
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "TEmperature degC");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[0].SHT11_data.temperature, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 3:	// Humidity SHT11
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Humidity    perc");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[0].SHT11_data.humidity, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 4:	// Dew point SHT11
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Dew point   degC");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[0].SHT11_data.dewpoint, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 5:	// Ambilight 1
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Ambilight 1  lux");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%lu", client_station[0].MAX44009_data.lux_ambilight_1);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 6:	// Ambilight 2
	{
		if(control.ok){

		}else if(control.plus){
			option[view] = 0;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Ambilight 2  lux");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%lu", client_station[0].MAX44009_data.lux_ambilight_2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	default:
		break;
	}
}

/**
 * OS Task: LCD Updater
 */
void vLCD_Update(void *pvParameters)
{
	volatile LCDUpTaskParam *pxTaskParam;
	pxTaskParam = (LCDUpTaskParam *) pvParameters;
	TickType_t pxPreviousWakeTime;
//	char lcd_buff[16];
	uint8_t prev_screen = 0;
//	uint8_t prev_option = 0;
//	size_t prevlen;

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
		time = (uint32_t)(millis()/1000);
		if((prev_screen != view)){
			prev_screen = view;
//			prev_option = option[view];
			LCDI2C_clear();
		}

		switch(view){
		case LCD_VIEW_MAIN:
		{
			viewMain(&lcd_view_controls);
			break;
		}
		case LCD_VIEW_SETTINGS:
		{
			view = LCD_VIEW_MAIN;
			viewMain(&lcd_view_controls);
			break;
		}
		case LCD_VIEW_LOAD_DATA:
		{
			viewLoadData(&lcd_view_controls);
			break;
		}
		case LCD_VIEW_VIEW_DATA:
		{
			viewViewData(&lcd_view_controls);
			break;
		}
		case LCD_VIEW_SYNC_TIME:
		{
			view = LCD_VIEW_MAIN;
			viewMain(&lcd_view_controls);
			break;
		}
		default:
			break;
		}

		lcd_view_controls.ok = 0;
		lcd_view_controls.plus = 0;
		lcd_view_controls.minus = 0;
		lcd_view_controls.back = 0;

//		if(prev_screen != pxTaskParam->view){
//			prev_screen = pxTaskParam->view;
//			LCDI2C_clear();
//		}else{
//		}
//
//		switch(pxTaskParam->view)
//		{
//		case 0:
//		{
//			// Display time since boot [ms]
//			memset(lcd_buff, '\0', 16);
//			sprintf(lcd_buff, "MS: %lu", millis());
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_setCursor(0, 0);
//			LCDI2C_write_String(lcd_buff);
//
//			// Display SHT11 dew point [degC]
//			memset(lcd_buff, '\0', 16);
//			sprintf(lcd_buff, "D: ");
//			LCDI2C_setCursor(0, 1);
//			LCDI2C_write_String(lcd_buff);
//
//			memset(lcd_buff, '\0', 16);
//			LCDI2C_setCursor(3, 1);
//			ftoa(pxTaskParam->dewpoint, lcd_buff, 2);
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_write_String(lcd_buff);
//
//			break;
//		}
//		case 1:
//		{
//			// Display temprature [degC]
//			memset(lcd_buff, '\0', 16);
//			sprintf(lcd_buff, "T: ");
//			LCDI2C_setCursor(0, 0);
//			LCDI2C_write_String(lcd_buff);
//
//			memset(lcd_buff, '\0', 16);
//			LCDI2C_setCursor(3, 0);
//			ftoa(pxTaskParam->temp, lcd_buff, 2);
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_write_String(lcd_buff);
//
//			// Display pressure [pa]
//			memset(lcd_buff, '\0', 16);
//			LCDI2C_setCursor(0, 1);
//			sprintf(lcd_buff, "P: %lu", pxTaskParam->press);
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_write_String(lcd_buff);
//			break;
//		}
//		case 2:
//		{
//			// Display 1st ambient light [lx]
//			memset(lcd_buff, '\0', 16);
//			sprintf(lcd_buff, "L1: %lu", pxTaskParam->lux_ambilight_1);
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_setCursor(0, 0);
//			LCDI2C_write_String(lcd_buff);
//
//			// Display 2nd ambient light [lx]
//			memset(lcd_buff, '\0', 16);
//			sprintf(lcd_buff, "L2: %lu", pxTaskParam->lux_ambilight_2);
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_setCursor(0, 1);
//			LCDI2C_write_String(lcd_buff);
//			break;
//		}
//		case 3:
//		{
//			// Display SHT11 temperature [degC]
//			memset(lcd_buff, '\0', 16);
//			sprintf(lcd_buff, "T: ");
//			LCDI2C_setCursor(0, 0);
//			LCDI2C_write_String(lcd_buff);
//
//			memset(lcd_buff, '\0', 16);
//			LCDI2C_setCursor(3, 0);
//			ftoa(pxTaskParam->sht_temp, lcd_buff, 2);
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_write_String(lcd_buff);
//
//			// Display SHT11 humidity [percent]
//			memset(lcd_buff, '\0', 16);
//			sprintf(lcd_buff, "H: ");
//			LCDI2C_setCursor(0, 1);
//			LCDI2C_write_String(lcd_buff);
//
//			memset(lcd_buff, '\0', 16);
//			LCDI2C_setCursor(3, 1);
//			ftoa(pxTaskParam->humidity, lcd_buff, 2);
//			prevlen = strlen(lcd_buff);
//			memset(lcd_buff + prevlen, ' ', 16 - prevlen);
//			LCDI2C_write_String(lcd_buff);
//			break;
//		}
//		default:
//			break;
//		}

		vTaskDelayUntil(&pxPreviousWakeTime, pxTaskParam->period);
	}while(1);
	vTaskDelete(NULL);
}


#endif /* VTASKLCD_H_ */
