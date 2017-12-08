#ifndef INC_H_
#define INC_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diag/Trace.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "common_freertos.h"
#include "UART_freertos.h"
#include "ssd1306.h"
#include "mc_indicator.h"
#include "esp8266_simple.h"

#endif /* INC_H_ */
