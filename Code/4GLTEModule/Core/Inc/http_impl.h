/*
 * http_impl.h
 *
 *  Created on: Feb 10, 2024
 *      Author: kan_a
 */

#ifndef INC_HTTP_IMPL_H_
#define INC_HTTP_IMPL_H_

#include <string.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l432xx.h"

extern UART_HandleTypeDef huart1;
extern USART_HandleTypeDef husart2;

void Tx_Rx(char *cmd);
void check_status(void);
void http_get(void);
void http_post(void);

#endif /* INC_HTTP_IMPL_H_ */
