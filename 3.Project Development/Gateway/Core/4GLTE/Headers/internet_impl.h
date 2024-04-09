/*
 * internet_impl.c
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 *	Contains function prototypes necessary for managing TCP/IP services
 *
 */


#ifndef INC_INTERNET_IMPL_H_
#define INC_INTERNET_IMPL_H_

#include <string.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l432xx.h"
#include "send_data.h"


void start_service(void);
void stop_service(void);
void get_ipaddr(void);


#endif /* INC_INTERNET_IMPL_H_ */
