/*
 * vTaskLCD.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 * Last change: Mar 29, 2018
 *
 */

#ifndef VTASKLCD_H_
#define VTASKLCD_H_
/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define NUM_VIEWS	 			5

#define LCD_VIEW_MAIN 			0
#define LCD_VIEW_SETTINGS		1
#define LCD_VIEW_LOAD_DATA		2
#define LCD_VIEW_VIEW_DATA		3
#define LCD_VIEW_SYNC_TIME		4

#define DIGIT_BLINK_PERIOD 		250
/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
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
uint8_t option[NUM_VIEWS];
uint8_t working_station = 0;

uint32_t time = 0;
uint32_t digit_blink_timer = 0;
uint8_t digit_blink = 0;		// 1 - display value; 0 - display spaces;
uint8_t set_mode = 0; 			// 0 - view navigation mode; 1 - value adjusting mode;
uint8_t save_settings = 0;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void viewMain(view_controls_t *view_controls);			// LCD_VIEW_MAIN 		= 0
void viewSettings(view_controls_t *view_controls); 		// LCD_VIEW_SETTINGS 	= 1
void viewLoadData(view_controls_t *view_controls);		// LCD_VIEW_LOAD_DATA 	= 2
void viewViewData(view_controls_t *view_controls);		// LCD_VIEW_VIEW_DATA 	= 3
//void viewSyncTime(view_controls_t *view_controls);		// LCD_VIEW_SYNC_TIME 	= 4
void vLCD_Update(void *pvParameters);					// OS Task

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

		uint8_t _stations_connected = 0;
		if((xStationsConnectedSemaphore != NULL) && (xSemaphoreTake(xStationsConnectedSemaphore, 0) == pdTRUE)){
			_stations_connected = stations_connected;
			memset(view_buffer, '\0', 16);
			sprintf(view_buffer, "STA: %d", _stations_connected);
			prevlen = strlen(view_buffer);
			memset(view_buffer + prevlen, ' ', 16 - prevlen);
			LCDI2C_setCursor(0, 1);
			LCDI2C_write_String(view_buffer);
			xSemaphoreGive(xStationsConnectedSemaphore);
		}
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

void viewSettings(view_controls_t *view_controls)
{
	view_controls_t control;
	control = *view_controls;
	char view_buffer[16];

	switch(option[view]){
	case 0:	// Set working station
	{
		if(control.ok){
			option[view] = 1;
		}else if(control.plus){
			if(working_station < 4){
				working_station++;
			}else{
				working_station = 0;
			}
		}else if(control.minus){
			if(working_station > 0){
				working_station--;
			}else{
				working_station = 4;
			}
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Select station  ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		if(client_station[working_station].network.link_status == espLinkOk){
			sprintf(view_buffer, " =%d         OK  ", working_station);
		}else{
			sprintf(view_buffer, " =%d         NO  ", working_station);
		}

		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);

		break;
	}
	case 1:		// Set main log period
	{
		if(control.ok){
			if(set_mode == 1){
				set_mode = 0;
			}else{
				set_mode = 1;
			}
		}else if(control.plus){
			if(set_mode == 1){
				client_station[working_station].settings.logging_period++;	// change value ++ [minutes]
			}else{
				option[view] = 2; // view navigation
			}
			option[view] = 2;
		}else if(control.minus){
			if(set_mode == 1){
				client_station[working_station].settings.logging_period--;	// change value --  [minutes]
			}else{
				option[view] = 2; // view navigation
			}
		}else if(control.back){
			option[view] = 3;
		}

		if(set_mode == 1){ 		// use blinking digits if we are in "set" mode
			if((millis() - digit_blink_timer) >= DIGIT_BLINK_PERIOD){
				if(digit_blink == 1){
					digit_blink = 0;
				}else{
					digit_blink = 1;
				}
				digit_blink_timer = millis();
			}
		}else{
			digit_blink = 1;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Set log period  ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		if(digit_blink == 1){
			sprintf(view_buffer, " =%d         MIN ", client_station[working_station].settings.logging_period);
		}else{
			sprintf(view_buffer, " =          MIN ");
		}

		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 2:		// Set 1Hz log enale
	{
		if(control.ok){				// press "ok" for changing mode between "set" and "view navigation"
			if(set_mode == 1){
				set_mode = 0;
			}else{
				set_mode = 1;
			}
		}else if(control.plus){
			if(set_mode == 1){
				client_station[working_station].settings.enable_ihz_log = 1;	// change value ++ [minutes]
			}else{
				option[view] = 2; // view navigation
			}
			option[view] = 2;
		}else if(control.minus){
			if(set_mode == 1){
				client_station[working_station].settings.enable_ihz_log = 0;	// change value --  [minutes]
			}else{
				option[view] = 2; // view navigation
			}
		}else if(control.back){
			option[view] = 3;
		}

		if(set_mode == 1){ 			// use blinking digits if we are in "set" mode
			if((millis() - digit_blink_timer) >= DIGIT_BLINK_PERIOD){
				if(digit_blink == 1){
					digit_blink = 0;
				}else{
					digit_blink = 1;
				}
				digit_blink_timer = millis();
			}
		}else{
			digit_blink = 1;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Enable 1Hz log  ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		if(client_station[working_station].settings.enable_ihz_log == 1){
			if(digit_blink == 1){
				sprintf(view_buffer, " = YES          ");
			}else{
				sprintf(view_buffer, " =              ");
			}
		}else{
			if(digit_blink == 1){
				sprintf(view_buffer, " = NO          ");
			}else{
				sprintf(view_buffer, " =              ");
			}
		}

		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 3:		// Save settings
	{
		if(control.ok){				// Send current values to the station
			// FIXME: STUB; add saving code
			view = LCD_VIEW_MAIN;
		}else if(control.plus){
			save_settings = 1;		// YES
		}else if(control.minus){
			save_settings = 0;		// NO
		}else if(control.back){
			option[view] = 1;		// CANCEL
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Save settings?  ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		if(save_settings == 1){
			if(digit_blink == 1){
				sprintf(view_buffer, " = YES          ");
			}else{
				sprintf(view_buffer, " =              ");
			}
		}else{
			if(digit_blink == 1){
				sprintf(view_buffer, " = NO          ");
			}else{
				sprintf(view_buffer, " =              ");
			}
		}

		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	default:
		break;
	}
}

void viewLoadData(view_controls_t *view_controls)
{
	view_controls_t control;
	control = *view_controls;
	char view_buffer[16];
	size_t prevlen;

	switch(option[view]){
	case 0:	// Set working station
	{
		if(control.ok){
			option[view] = 1;
		}else if(control.plus){
			if(working_station < 4){
				working_station++;
			}else{
				working_station = 0;
			}
		}else if(control.minus){
			if(working_station > 0){
				working_station--;
			}else{
				working_station = 4;
			}
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Select station  ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		if(client_station[working_station].network.link_status == espLinkOk){
			sprintf(view_buffer, " =%d         OK  ", working_station);
		}else{
			sprintf(view_buffer, " =%d         NO  ", working_station);
		}

		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);

		break;
	}
	case 1: // Load logs
	{
		if(control.ok){
			if(client_station[working_station].network.link_status == espLinkOk){
				client_station[working_station].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_LIST;
				option[view] = 5;
			}else{

			}
		}else if(control.plus){
			option[view] = 2;
		}else if(control.minus){
			option[view] = 2;
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
	case 2:	// Delete logs
	{
		if(control.ok){
			option[view] = 3;
		}else if(control.plus){
			option[view] = 1;
		}else if(control.minus){
			option[view] = 1;
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
	case 3:	// Are you sure?
	{
		if(control.ok){
			view = LCD_VIEW_MAIN;	// STUB // Eracing command
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
	case 4:	// Deleting
	{
		if(control.ok){

		}else if(control.plus){

		}else if(control.minus){

		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "    Deleting    ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%d of %d",
				client_station[working_station].log_file_op.log_file_cnt,
				client_station[working_station].log_file_op.log_file_seq);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 5:	// Loading
	{
		if(control.ok){

		}else if(control.plus){

		}else if(control.minus){

		}else if(control.back){
			option[view] = 1;
			// Cancel
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "    Loading     ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%d of %d",
				client_station[working_station].log_file_op.log_file_cnt,
				client_station[working_station].log_file_op.log_file_seq);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 6:	// Done
	{
		if(control.ok){
			option[view] = 1;
		}else if(control.plus){
			option[view] = 1;
		}else if(control.minus){
			option[view] = 1;
		}else if(control.back){
			option[view] = 1;
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
	case 0:	// Set working station
	{
		if(control.ok){
			option[view] = 1;
		}else if(control.plus){
			if(working_station < 4){
				working_station++;
			}else{
				working_station = 0;
			}
		}else if(control.minus){
			if(working_station > 0){
				working_station--;
			}else{
				working_station = 4;
			}
		}else if(control.back){
			view = LCD_VIEW_MAIN;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Select station  ");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		if(client_station[working_station].network.link_status == espLinkOk){
			sprintf(view_buffer, " =%d         OK  ", working_station);
		}else{
			sprintf(view_buffer, " =%d         NO  ", working_station);
		}

		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);

		break;
	}
	case 1: // Temperature BMP180
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view] = 7;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Temperature degC");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[working_station].BMP180_data.temperature, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);

		break;
	}
	case 2:	// Pressure BMP180
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Pressure      pa");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%lu", client_station[working_station].BMP180_data.pressure);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 3:	// Temperature SHT11
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "TEmperature degC");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[working_station].SHT11_data.temperature, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 4:	// Humidity SHT11
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Humidity    perc");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[working_station].SHT11_data.humidity, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 5:	// Dew point SHT11
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Dew point   degC");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		ftoa(client_station[working_station].SHT11_data.dewpoint, view_buffer, 2);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 6:	// Ambilight 1
	{
		if(control.ok){

		}else if(control.plus){
			option[view]++;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Ambilight 1  lux");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%lu", client_station[working_station].MAX44009_data.lux_ambilight_1);
		prevlen = strlen(view_buffer);
		memset(view_buffer + prevlen, ' ', 16 - prevlen);
		LCDI2C_setCursor(0, 1);
		LCDI2C_write_String(view_buffer);
		break;
	}
	case 7:	// Ambilight 2
	{
		if(control.ok){

		}else if(control.plus){
			option[view] = 1;
		}else if(control.minus){
			option[view]--;
		}else if(control.back){
			option[view] = 0;
		}

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "Ambilight 2  lux");
		LCDI2C_setCursor(0, 0);
		LCDI2C_write_String(view_buffer);

		memset(view_buffer, '\0', 16);
		sprintf(view_buffer, "%lu", client_station[working_station].MAX44009_data.lux_ambilight_2);
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

//void viewSyncTime(view_controls_t *view_controls){
//	// FIXME: Add code here
//}

/**
 * OS Task: LCD Updater
 */
void vLCD_Update(void *pvParameters)
{
	volatile LCDUpTaskParam *pxTaskParam;
	pxTaskParam = (LCDUpTaskParam *) pvParameters;
	TickType_t pxPreviousWakeTime;
	uint8_t prev_screen = 0;

	LCDI2C_init(0x3F, 16, 2); // Init 16x2 LCD
	for(uint8_t i = 0; i < 3; i++) // Quick 3 blinks of backlight
	{
		LCDI2C_backlight();
		vTaskDelay(250);
		LCDI2C_noBacklight();
		vTaskDelay(250);
	}
	LCDI2C_backlight(); // finish with backlight on

	// NOTE: Cursor Position: (CHAR, LINE) start at 0
	LCDI2C_clear();

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		time = (uint32_t)(millis()/1000);
		if((prev_screen != view)){
			prev_screen = view;
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
			viewSettings(&lcd_view_controls);
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

		lcd_view_controls.ok = 0;		// Reset controls
		lcd_view_controls.plus = 0;
		lcd_view_controls.minus = 0;
		lcd_view_controls.back = 0;

		vTaskDelayUntil(&pxPreviousWakeTime, pxTaskParam->period);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKLCD_H_ */
