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
extern USART_HandleTypeDef husart2;

void start_service(void);
void stop_service(void);
void set_pdp(void);
void select_application_mode(void);
void set_send_mode(void);
void configure_param(void);
void set_timeout(void);
void get_ipaddr(void);
void setup_tcp_connection(void);
void destroy_tcp_connection(void);



#endif /* INC_INTERNET_IMPL_H_ */
