/*
 * led_impl.c
 *
 * Author: Salman Shuaib
 *
 * Description:
 *
 * This file contains functions for turning the STM'S on board LED on and off
 * This LED is used for signalling purposes.
 *
 */
#include "led.h"


/**
 * This function configures the required GPIO clock and pins
 * to either input or output.
 *
 * Parameters:
 * 	Null
 *
 * Returns:
 * 	Null
 */
void initGPIO(void)
{
	//enable clock for GPIO B
	RCC->AHB2ENR |= (1U << 1);
	//RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	//set bits 6,7,8 and 9 to 0. Since bits 8 and 9 were set to 0, PB_4 already set to input mode
	GPIOB->MODER &= ~(15U << 6);

	//set PB_3 to output mode
	GPIOB->MODER |= (1U << 6);
}

/**
 * Turns on board led(PB3) on
 *
 * Parameters:
 * 	Null
 *
 * Returns:
 * 	Null
 */
void boardled_on(void)
{
		GPIOB->ODR |= (1U << 3);
}

/**
 * Turns on board led(PB3) off
 *
 * Parameters:
 * 	Null
 *
 * Returns:
 * 	Null
 */
void boardled_off(void)
{
		GPIOB->ODR &= ~(1U << 3);
}



