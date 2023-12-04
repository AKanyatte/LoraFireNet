#ifndef I2C_H
#define I2C_H

#include "stm32l4xx.h"

// Function prototypes
void I2C_Init(void);
void I2C_Write(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data);
uint8_t I2C_Read(uint8_t deviceAddress, uint8_t registerAddress);

#endif // I2C_H
