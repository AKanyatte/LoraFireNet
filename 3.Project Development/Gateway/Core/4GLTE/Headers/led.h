/*
 * led_impl.h
 *
 * Author: Salman Shuaib
 *
 * Description:
 *
 * This file contains function prototypes necessary for for turning the
 * STM'S on board LED on and off
 *
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32l432xx.h"

void initGPIO(void);
void boardled_on(void);
void boardled_off(void);
void delay(uint32_t);


#endif /* INC_LED_H_ */
