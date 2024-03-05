#include "stm32l432xx.h"

void initGPIO(void);
void boardled_on(void);
void boardled_off(void);
void delay(uint32_t);

/*****************************************************************
* initGPIO
*
* This function configures the required GPIO clock and pins
* to either input or output.
*****************************************************************/

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

int main(void)
{
	int btnPressed = 0;		//state of button pin (PB_4) saved here

	//initialise GPIO
	initGPIO();
	
	//loop around checking if button was pressed.
	while(!btnPressed)
	{
		//inspect the state of the button pin by checking bit 4 of GPIOB_IDR
		btnPressed = (GPIOB->IDR & (1U << 4));
		
		while(1)
		{
			//button was pressed, turn LED on
			boardled_on();
			delay(100000);
			boardled_off();
			delay(100000);
		}

	}
}
