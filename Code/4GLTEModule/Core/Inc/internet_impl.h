/*
 * internet_impl.h
 *
 *  Created on: Feb 3, 2024
 *      Author: kan_a
 */

#ifndef INC_INTERNET_IMPL_H_
#define INC_INTERNET_IMPL_H_

#include <string.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l432xx.h"

extern UART_HandleTypeDef huart1;

void start_service(void);
void stop_service(void);
void access_server(void);


#endif /* INC_INTERNET_IMPL_H_ */
