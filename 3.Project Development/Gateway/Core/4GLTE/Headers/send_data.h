/*
 * send_data.h
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 * Function prototypes necessary for SMS configurations and defined constants.
 *
 */

#ifndef INC_SEND_DATA_H_
#define INC_SEND_DATA_H_

#include <string.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l432xx.h"

#define MILD_CHANCE			60
#define HIGH_LIKELIHOOD		70
#define IMMINENT_WILDFIRE	80
#define HIGHLY_PROBABLE		90

extern UART_HandleTypeDef huart1;
extern USART_HandleTypeDef husart2;
extern uint8_t buffer[1000];

void Tx_Rx(char *cmd);
void send_at(void);
void set_sms_mode(void);
void send_sms1(void);
void send_sms(char *alert);
void send_alert(void);

#endif /* INC_SEND_DATA_H_ */
