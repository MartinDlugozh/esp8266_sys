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
#include "xfprintf/xprintf.h"

// Drivers
#include "blue_pill.h"
//#include "ssd1306.h"		// SSD1306 I2C OLED display driver
#include "LiquidCrystal_I2C.h"
#include "mc_indicator.h" 			// ECOLOG motherboard v.0.1a LED indicator
#include "esp8266_simple.h"
//#include "esp8266.h"

#include "server_global_sect.h"

// MAVLink headers
#include "./mavlink_eco/eco_messages/mavlink.h"
#include "./mavlink_eco/protocol.h"
#include "./mavlink_eco/mavlink_helpers.h"
#include "./mavlink_eco/checksum.h"
#include "./mavlink_eco/mavlink_types.h"
#include "./mavlink_eco/mavlink_hlp_esp8266.h"

// Tasks
#include "Tasks/vTaskBlinker.h"
#include "Tasks/vTaskLCD.h"
#include "Tasks/vTaskButtonsCheck.h"
#include "Tasks/vTaskESP.h"
#include "Tasks/vTaskSDcard.h"

#endif /* INC_H_ */
