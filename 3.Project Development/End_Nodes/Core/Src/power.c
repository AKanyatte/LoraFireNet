/*
 * File name: power.c
 *
 * This file contains functions to control the done pin on the TPL5111.
 * The done pin signals to the TPL5111 that the driven electronics have completed their task.
 * When the done pin is set high, it indicates that the task is complete, and the TPL5111 can turn off.
 *
 * Author: Salman Shuaib
 */

#include "power.h"

/**
 * This function sets the done pin to a low state.
 */
void set_done_low(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
 * This function sets the done pin to a high state.
 */
void set_done_high(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}
