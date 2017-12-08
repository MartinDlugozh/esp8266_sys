#ifndef COMMON_FREERTOS_H_
#define COMMON_FREERTOS_H_
#pragma once

#include "inc.h"

inline uint32_t millis(void)
{
   return (uint32_t)xTaskGetTickCount();
}

void delay (int timeout)
{
	uint32_t timeIn = millis();
	while(timeIn + timeout > millis());
}


#endif /* COMMON_FREERTOS_H_ */
