/*
 * mc_indicator.h
 *
 * Created: 13.11.2017 10:00:19
 *  Author: Dr. Saldon
 *
 *
 *  LED Indicator convinence functions
 *  Used on ECOLOG motherboard v.0.1a
 */

#ifndef MC_INDICATOR_H_
#define MC_INDICATOR_H_
#pragma once

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define LED_PORT_RED1 		GPIOB
#define LED_PORT_RED2	 	GPIOB
#define LED_PORT_GREEN1 	GPIOB
#define LED_PORT_GREEN2 	GPIOB

#define LED_PORT_1 			LED_PORT_GREEN1
#define LED_PORT_2 			LED_PORT_GREEN2
#define LED_PORT_3 			LED_PORT_RED1
#define LED_PORT_4 			LED_PORT_RED2

#define LED_PIN_GREEN1 		GPIO_Pin_12
#define LED_PIN_GREEN2 		GPIO_Pin_13
#define LED_PIN_RED1 		GPIO_Pin_14
#define LED_PIN_RED2 		GPIO_Pin_15

#define LED_PIN_1 			LED_PIN_GREEN1
#define LED_PIN_2 			LED_PIN_GREEN2
#define LED_PIN_3 			LED_PIN_RED1
#define LED_PIN_4 			LED_PIN_RED2

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// General STM32 includes
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void led_indicator_init(void);
void set_led_status(uint8_t status);
void led_ppmm(uint8_t *led_cnt , uint8_t inc);
void toggle_led1(void);
void toggle_led2(void);
void toggle_led3(void);
void toggle_led4(void);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

#endif /* MC_INDICATOR_H_ */
