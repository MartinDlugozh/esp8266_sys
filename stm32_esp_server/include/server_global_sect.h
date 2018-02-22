/*
 * server_global_sect.h
 *
 *  Created on: Jan 28, 2018
 *      Author: Dr. Saldon
 */

#ifndef INCLUDE_SERVER_GLOBAL_SECT_H_
#define INCLUDE_SERVER_GLOBAL_SECT_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define NUM_CLIENT_STATIONS 	1

#define SYS_ID_MY 		254
#define COM_ID_MY 		0
#define SYS_ID_GCS 		255

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
uint8_t mavlink_enable = 1; 		// Use MAVLink protocol for telemetry
uint8_t led_cnt = 0;

typedef struct _HEARTBEAT_data{
	uint8_t system_id;
	uint8_t componnent_id;
} HEARTBEAT_data_t;

typedef struct _SYSTEMTIME_data{
	uint32_t time_boot_ms;
} SYSTEMTIME_data_t;

typedef struct _BMP180_data{
	float temterature;
	float altitude;
	uint32_t pressure;
} BMP180_data_t;

BMP180_data_t BMP180_data;

typedef struct _MAX44009_data{
	uint32_t lux_ambilight_1;
	uint32_t lux_ambilight_2;
} MAX44009_data_t;

typedef struct _SHT11_data{
	float 		temperature;
	float 		humidity;
	float 		dewpoint;
} SHT11_data_t;

typedef struct _client_station{
	HEARTBEAT_data_t 	HEARTBEAT_data;
	SYSTEMTIME_data_t 	SYSTEMTIME_data;
	MAX44009_data_t 	MAX44009_data;
	SHT11_data_t 		SHT11_data;
	BMP180_data_t 		BMP180_data;
} client_station_t;

client_station_t client_station[NUM_CLIENT_STATIONS];

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/


#endif /* INCLUDE_SERVER_GLOBAL_SECT_H_ */
