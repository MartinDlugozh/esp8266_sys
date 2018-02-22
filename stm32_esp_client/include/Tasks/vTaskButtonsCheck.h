/*
 * vTaskButtonsCheck.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKBUTTONSCHECK_H_
#define VTASKBUTTONSCHECK_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// General STM32 includes
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "blue_pill/blue_pill.h"
//#include "esp8266_simple.h"			// Simple ESP8266 driver

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
#define BOARD_SERVICE_BUTTON_PIN		GPIO_Pin_1
#define BOARD_SERVICE_BUTTON_PORT		GPIOB

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void buttonPressedHandle(void);
void vButtonsCheck(void *pvParameters);
void initBoardButtons(void);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void initBoardButtons(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Pin = BOARD_SERVICE_BUTTON_PIN;
	GPIO_Init(BOARD_SERVICE_BUTTON_PORT, &gpio);
}

void serviceButtonPressedHandle(void)
{
	// send increment over TCP
//	unsigned char c[2] = {0, 0};
//	c[0] = '+';
//	esp8266TcpSend(c, 1, 1);
//	led_ppmm(&led_cnt, 1); // increment led counter
}

/**
 * OS Task: Button check
 *
 * Checks state of the onboard pushbutton (PB11)
 * Uses buttonPressedHandle() method
 */
void vButtonsCheck(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;
	uint8_t btn_state_curr = 0;		// previous button state
	uint8_t btn_state_prev = 0;		// current button state
	uint16_t period = 20;			// checking period

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		btn_state_prev = btn_state_curr;		// shift current state to previous
		btn_state_curr = GPIO_ReadInputDataBit(BOARD_SERVICE_BUTTON_PORT, BOARD_SERVICE_BUTTON_PIN);		// get new state
		if((btn_state_curr != 0) && (btn_state_curr != btn_state_prev))	// if state had changed to "PRESSED"
		{
			period = 400;		// change checking period (debounce)
			serviceButtonPressedHandle();
		}else if((btn_state_curr != 0) && (btn_state_curr == btn_state_prev)){
			period = 200;		// change checking period (debounce)
			serviceButtonPressedHandle();
		}else{
			period = 20;		// change checking period back for normal reaction
		}

		vTaskDelayUntil(&pxPreviousWakeTime, period);
	}while(1);
	vTaskDelete(NULL);
}



#endif /* VTASKBUTTONSCHECK_H_ */
