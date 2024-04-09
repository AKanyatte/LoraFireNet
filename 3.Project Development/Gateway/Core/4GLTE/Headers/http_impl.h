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
#include <stdlib.h>
#include "stm32l4xx_hal.h"
#include "stm32l432xx.h"
#include "send_data.h"


//Structure to hold prediction data
typedef struct {
    float predicted_probability;
} PredictionData;

void check_status(void);
float get_prediction(void);
void http_get(void);
void http_post(uint8_t humidity_val, uint8_t smoke_val, uint8_t temp_val);

#endif /* INC_HTTP_IMPL_H_ */
