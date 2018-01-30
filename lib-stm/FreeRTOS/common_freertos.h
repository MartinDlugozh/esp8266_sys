#ifndef COMMON_FREERTOS_H_
#define COMMON_FREERTOS_H_
#pragma once

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
inline uint32_t millis(void);
void delay (int timeout);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * Milliseconds
 *
 * Use it only inside the FreeRTOS tasks bodies
 */
inline uint32_t millis(void)
{
   return (uint32_t)xTaskGetTickCount();
}

/**
 * Delay (default)
 *
 * Yes, this is that damn delay function
 */
//void delay(int timeout)
//{
//	uint32_t timeIn = millis();
//	while(timeIn + timeout > millis());
//}

#endif /* COMMON_FREERTOS_H_ */
