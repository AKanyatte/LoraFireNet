/*
 * send_data.h
 *
 *  Created on: Feb 3, 2024
 *      Author: kan_a
 */

#ifndef INC_SEND_DATA_H_
#define INC_SEND_DATA_H_

#include <string.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l432xx.h"

extern UART_HandleTypeDef huart1;

void send_at(void);
void set_sms_mode(void);
void send_sms(void);

#endif /* INC_SEND_DATA_H_ */
