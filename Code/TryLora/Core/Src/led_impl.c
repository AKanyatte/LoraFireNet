/*
 * led_impl.c
 *
 *  Created on: Feb 1, 2024
 *      Author: kan_a
 */

#include "led.h"


/*****************************************************************
* initGPIO
*
* This function configures the required GPIO clock and pins
* to either input or output.
*****************************************************************/
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

void boardled_on(void)
{
		GPIOB->ODR |= (1U << 3);
}

void boardled_off(void)
{
		GPIOB->ODR &= ~(1U << 3);
}

void delay(uint32_t delay)
{
  	 while (delay--)
	{
	}
}

