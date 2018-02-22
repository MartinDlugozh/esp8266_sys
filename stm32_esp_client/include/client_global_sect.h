/*
 * client_global_sect.h
 *
 *  Created on: Jan 24, 2018
 *      Author: Dr. Saldon
 */

#ifndef CLIENT_GLOBAL_SECT_H_
#define CLIENT_GLOBAL_SECT_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define SYS_ID_MY 		1
#define COM_ID_MY 		0
#define SYS_ID_GCS 		255

#define SENSOR_DEAD 	0
#define SENSOR_OK 		1
#define SENSOR_CHECK	2

#define FU1_printf(fmt, ...) {	if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{xprintf(fmt, __VA_ARGS__); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
uint8_t mavlink_enable = 1; 		// Use MAVLink protocol for telemetry
uint8_t led_cnt = 0;

typedef struct _BMP180_data{
	float 		temterature;
	float 		altitude;
	uint32_t 	pressure;
} BMP180_data_t;

typedef struct _MAX44009_data{
	uint32_t 	lux_ambilight_1;
	uint32_t 	lux_ambilight_2;
} MAX44009_data_t;

typedef struct _SHT11_data{
	float 		temperature;
	float 		humidity;
	float 		dewpoint;
} SHT11_data_t;

typedef struct _SYSTEMTIME_data{
	uint32_t 	time_boot_ms;
} SYSTEMTIME_data_t;

typedef struct _SERVER_data{
	uint32_t 	last_heartbeat_ms;
} SERVER_data_t;

typedef struct _station{
	SYSTEMTIME_data_t 	SYSTEMTIME_data;
	SERVER_data_t 		SERVER_data;
	MAX44009_data_t 	MAX44009_data;
	SHT11_data_t 		SHT11_data;
	BMP180_data_t 		BMP180_data;
} station_t;

station_t station;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/


#endif /* CLIENT_GLOBAL_SECT_H_ */
