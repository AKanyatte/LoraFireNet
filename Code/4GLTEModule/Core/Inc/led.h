/*
 * led.h
 *
 *  Created on: Feb 1, 2024
 *      Author: kan_a
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32l432xx.h"

void initGPIO(void);
void boardled_on(void);
void boardled_off(void);
void delay(uint32_t);


#endif /* INC_LED_H_ */
