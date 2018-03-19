/*
 * vTaskButtonsCheck.h
 *
 *  Created on: Mar 11, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKBUTTONSCHECK_H_
#define VTASKBUTTONSCHECK_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define BUTTONS_TASK_PERIOD 			10
#define BUTTONS_COUNT 					4

//#define BOARD_BUTTON_SERVICE			0
//#define BOARD_BUTTON_OK 				1
//#define BOARD_BUTTON_PLUS 				2
//#define BOARD_BUTTON_MINUS 				3
//#define BOARD_BUTTON_BACK 				4

#define BOARD_BUTTON_OK 				0
#define BOARD_BUTTON_PLUS 				1
#define BOARD_BUTTON_MINUS 				2
#define BOARD_BUTTON_BACK 				3

//#define BOARD_BUTTON_SERVICE_PIN		GPIO_Pin_11
//#define BOARD_BUTTON_SERVICE_PORT		GPIOB

#define BOARD_BUTTON_OK_PIN				GPIO_Pin_1		// white
#define BOARD_BUTTON_OK_PORT			GPIOA

#define BOARD_BUTTON_PLUS_PIN			GPIO_Pin_0		// green
#define BOARD_BUTTON_PLUS_PORT			GPIOA

#define BOARD_BUTTON_MINUS_PIN			GPIO_Pin_15		// yellow
#define BOARD_BUTTON_MINUS_PORT			GPIOC

#define BOARD_BUTTON_BACK_PIN			GPIO_Pin_14		// orange
#define BOARD_BUTTON_BACK_PORT			GPIOC

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
#include "blue_pill.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
TaskHandle_t xTaskButtonsHandle;

typedef struct _BUTTON{
	uint8_t stage;
	uint32_t rise_time;
}BUTTON_bt;

uint32_t pressed_time = 0;
BUTTON_bt btn_sta[BUTTONS_COUNT];
//typedef struct _button_state{
//	uint8_t current;		// previous button state
//	uint8_t previous;		// current button state
//} button_state_t;
//
//button_state_t 	board_btn_state[BOARD_BTN_CNT];

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
//void initBoardButtons(void);
//void serviceButtonPressedHandle(void);
//void okButtonPressedHandle(void);
//void plusButtonPressedHandle(void);
//void minusButtonPressedHandle(void);
//void backButtonPressedHandle(void);
//void updateButton(uint8_t button, GPIO_TypeDef* port, uint16_t pin, uint16_t *period, void (*handle)(void));
//void vButtonsCheck(void *pvParameters);
void initButtons(void);
void okButtonPressedHandle(void);
void plusButtonPressedHandle(void);
void minusButtonPressedHandle(void);
void backButtonPressedHandle(void);
void buttonStateUpdate(BUTTON_bt *btn_sta, void (*handle)(void));
void vButtonsCheck(void *pvParameters);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

//void initBoardButtons(void)
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
//							RCC_APB2Periph_GPIOB |
//							RCC_APB2Periph_GPIOC,
//							ENABLE);
//
//	GPIO_InitTypeDef gpio;
//	GPIO_StructInit(&gpio);
//	gpio.GPIO_Mode = GPIO_Mode_IPD;
//	gpio.GPIO_Speed = GPIO_Speed_2MHz;
//
//	gpio.GPIO_Pin = BOARD_BUTTON_SERVICE_PIN;
//	GPIO_Init(BOARD_BUTTON_SERVICE_PORT, &gpio);
//
//	gpio.GPIO_Pin = BOARD_BUTTON_OK_PIN;
//	GPIO_Init(BOARD_BUTTON_OK_PORT, &gpio);
//
//	gpio.GPIO_Pin = BOARD_BUTTON_PLUS_PIN;
//	GPIO_Init(BOARD_BUTTON_PLUS_PORT, &gpio);
//
//	gpio.GPIO_Pin = BOARD_BUTTON_MINUS_PIN;
//	GPIO_Init(BOARD_BUTTON_MINUS_PORT, &gpio);
//
//	gpio.GPIO_Pin = BOARD_BUTTON_BACK_PIN;
//	GPIO_Init(BOARD_BUTTON_BACK_PORT, &gpio);
//
//	for(uint8_t i = 0; i < BOARD_BTN_CNT; i++){
//		board_btn_state[i].previous = 0;
//		board_btn_state[i].current = 0;
//	}
//}

void initButtons(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 	// OK, PLUS - PA1, PA0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	// BACK, MINUS - PC14, PC15
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* JTAG-DP Disabled and SW-DP Enabled */
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // needed for use PB3, PB4, PA15

	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; 		// PLUS - PA0
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;		// OK - PA1
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;		// BACK - PC14
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;		// MINUS - PC15
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);

    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;

    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_Init(&EXTI_InitStruct);
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    EXTI_Init(&EXTI_InitStruct);
    EXTI_InitStruct.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStruct);
    EXTI_InitStruct.EXTI_Line = EXTI_Line15;
    EXTI_Init(&EXTI_InitStruct);
}

//void serviceButtonPressedHandle(void)
//{
//	uint8_t lcd_screen;
//	lcd_screen = lcdParam.view;
//
//	lcd_screen++;
//	if(lcd_screen >= NUM_VIEWS_MAX){
//		lcd_screen = 0;
//	}
//	lcdParam.view = lcd_screen;
//}

void okButtonPressedHandle(void)
{
	lcd_view_controls.ok = 1;
}

void plusButtonPressedHandle(void)
{
	lcd_view_controls.plus = 1;
}

void minusButtonPressedHandle(void)
{
	lcd_view_controls.minus = 1;
}

void backButtonPressedHandle(void)
{
	lcd_view_controls.back = 1;
}

void buttonStateUpdate(BUTTON_bt *btn_sta, void (*handle)(void))
{
	if(btn_sta->stage > 0){
		switch(btn_sta->stage){
		case 1:
			if((millis() - btn_sta->rise_time) > pressed_time){
				btn_sta->stage++;
				pressed_time += 400;
				handle();
			}
		break;
		case 2:
			if((millis() - btn_sta->rise_time) > pressed_time){
				pressed_time += 250;
				handle();
			}
		break;
		default:
			break;
		}
	}
}

//void updateButton(uint8_t button, GPIO_TypeDef* port, uint16_t pin, uint16_t *period, void (*handle)(void))
//{
//	uint16_t _period;
//	_period = *period;
//
//	board_btn_state[button].previous = board_btn_state[button].current;
//	board_btn_state[button].current = GPIO_ReadInputDataBit(port, pin);
//
//	if((board_btn_state[button].current != 0) &&
//			(board_btn_state[button].current != board_btn_state[button].previous)){
//		_period = 400;		// change checking period (debounce)
//		handle();
//	}else if((board_btn_state[button].current != 0) &&
//			(board_btn_state[button].current == board_btn_state[button].previous)){
//		_period = 200;		// change checking period (debounce)
//		handle();
//	}
//
//	*period = _period;
//}

/**
 * OS Task: Button check
 *
 * Checks state of the onboard pushbutton (PB11)
 * Uses buttonPressedHandle() method
 */
void vButtonsCheck(void *pvParameters)
{
//	TickType_t pxPreviousWakeTime;
//	uint16_t period = 15;
//
//	pxPreviousWakeTime = xTaskGetTickCount();
//	do{
//		period = 15;
//		updateButton(BOARD_BUTTON_SERVICE, 	BOARD_BUTTON_SERVICE_PORT, 	BOARD_BUTTON_SERVICE_PIN, 	&period, serviceButtonPressedHandle);
//		updateButton(BOARD_BUTTON_OK, 		BOARD_BUTTON_OK_PORT, 		BOARD_BUTTON_OK_PIN, 		&period, okButtonPressedHandle);
//		updateButton(BOARD_BUTTON_PLUS, 	BOARD_BUTTON_PLUS_PORT, 	BOARD_BUTTON_PLUS_PIN, 		&period, plusButtonPressedHandle);
//		updateButton(BOARD_BUTTON_MINUS, 	BOARD_BUTTON_MINUS_PORT, 	BOARD_BUTTON_MINUS_PIN, 	&period, minusButtonPressedHandle);
//		updateButton(BOARD_BUTTON_BACK, 	BOARD_BUTTON_BACK_PORT, 	BOARD_BUTTON_BACK_PIN, 		&period, backButtonPressedHandle);
//
//		vTaskDelayUntil(&pxPreviousWakeTime, period);
//	}while(1);
//	vTaskDelete(NULL);
	TickType_t pxPreviousWakeTime;

	initButtons();

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		buttonStateUpdate(&btn_sta[BOARD_BUTTON_OK], 		okButtonPressedHandle);
		buttonStateUpdate(&btn_sta[BOARD_BUTTON_PLUS], 		plusButtonPressedHandle);
		buttonStateUpdate(&btn_sta[BOARD_BUTTON_MINUS], 	minusButtonPressedHandle);
		buttonStateUpdate(&btn_sta[BOARD_BUTTON_BACK], 		backButtonPressedHandle);

		vTaskDelayUntil(&pxPreviousWakeTime, BUTTONS_TASK_PERIOD);
	}while(1);
	vTaskDelete(NULL);
}

/**
 * plusButton IRQ Handler
 *
 * plusButton - PA0
 */
void EXTI0_IRQHandler(void) {
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
    	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1){
    		btn_sta[BOARD_BUTTON_PLUS].rise_time = millis();
    		btn_sta[BOARD_BUTTON_PLUS].stage = 1;
    		pressed_time = 20;
    	}else{
    		btn_sta[BOARD_BUTTON_PLUS].rise_time = 0;
    		btn_sta[BOARD_BUTTON_PLUS].stage = 0;
    		pressed_time = 0;
    	}
        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/**
 * okButton IRQ Handler
 *
 * okButton - PA1
 */
void EXTI1_IRQHandler(void) {
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
    	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1){
    		btn_sta[BOARD_BUTTON_OK].rise_time = millis();
    		btn_sta[BOARD_BUTTON_OK].stage = 1;
    		pressed_time = 20;
    	}else{
    		btn_sta[BOARD_BUTTON_OK].rise_time = 0;
    		btn_sta[BOARD_BUTTON_OK].stage = 0;
    		pressed_time = 0;
    	}
        EXTI_ClearITPendingBit(EXTI_Line1);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/**
 * minusButton and backButton IRQ Handler
 *
 * minusButton - PC15
 * backButton - PC14
 */
void EXTI15_10_IRQHandler(void) {
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

    if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
    	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == 1){
    		btn_sta[BOARD_BUTTON_BACK].rise_time = millis();
    		btn_sta[BOARD_BUTTON_BACK].stage = 1;
    		pressed_time = 20;
    	}else{
    		btn_sta[BOARD_BUTTON_BACK].rise_time = 0;
    		btn_sta[BOARD_BUTTON_BACK].stage = 0;
    		pressed_time = 0;
    	}
        EXTI_ClearITPendingBit(EXTI_Line14);
    }else if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
    	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 1){
    		btn_sta[BOARD_BUTTON_MINUS].rise_time = millis();
    		btn_sta[BOARD_BUTTON_MINUS].stage = 1;
    		pressed_time = 20;
    	}else{
    		btn_sta[BOARD_BUTTON_MINUS].rise_time = 0;
    		btn_sta[BOARD_BUTTON_MINUS].stage = 0;
    		pressed_time = 0;
    	}
    	EXTI_ClearITPendingBit(EXTI_Line15);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#endif /* VTASKBUTTONSCHECK_H_ */
