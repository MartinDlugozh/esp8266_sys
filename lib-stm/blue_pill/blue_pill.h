/*
 * blue_pill.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 */

#ifndef BLUE_PILL_H_
#define BLUE_PILL_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define BLUE_PILL_ONBOARD_LED_PIN		GPIO_Pin_13
#define BLUE_PILL_ONBOARD_LED_PORT		GPIOC

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// General STM32 includes
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void initBluePillLed(void);
void pin_toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void onboard_led_toggle(void);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void initBluePillLed(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Pin = BLUE_PILL_ONBOARD_LED_PIN;
	GPIO_Init(BLUE_PILL_ONBOARD_LED_PORT, &gpio);
}

/**
 * Toggle GPIO pin
 */
void pin_toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	uint8_t led_bit;
    /* Read LED output (GPIOA PIN8) status */
    led_bit = GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin);

    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(GPIOx, GPIO_Pin);
    }else/* If LED output clear, set it */
    {
        GPIO_SetBits(GPIOx, GPIO_Pin);
    }
}

/**
 * Toggle onboard led (PC13 on BluePill)
 */
void onboard_led_toggle(void)
{
	pin_toggle(BLUE_PILL_ONBOARD_LED_PORT, BLUE_PILL_ONBOARD_LED_PIN);
}

#endif /* BLUE_PILL_H_ */
