#ifndef INC_H_
#define INC_H_

// General C includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diag/Trace.h"

// Compiller rules
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

// General STM32 includes
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "common_freertos.h"					// Common convinence functions for FreeRTOS
#include "usart_freertos.h"		// USART convinence functions for FreeRTOS

#include "client_global_sect.h"

// Drivers
#include "blue_pill/blue_pill.h"
#include "mc_indicator.h" 						// ECOLOG motherboard v.0.1a LED indicator
#include "bmp180/bmp180.h" 						// BMP180 (pressure and temperature sensor) driver
#include "sht11_drv_sensibus/sht11_drv_sensibus.h"
#include "esp8266_simple.h"

// MAVLink headers
#include "mavlink_eco/eco_messages/mavlink.h"
#include "mavlink_eco/protocol.h"
#include "mavlink_eco/mavlink_helpers.h"
#include "mavlink_eco/checksum.h"
#include "mavlink_eco/mavlink_types.h"
#include "mavlink_eco/mavlink_hlp_esp8266.h"

// Tasks
#include "Tasks/vTaskButtonsCheck.h"
#include "Tasks/vTaskBMP180Sample.h"
#include "Tasks/vTaskMAX44009.h"
#include "Tasks/vTaskSHT11.h"
#include "Tasks/vTaskDebug.h"
#include "Tasks/vTaskBlinker.h"
#include "Tasks/vTaskSDcard.h"
#include "Tasks/vTaskESP.h"

#endif /* INC_H_ */
