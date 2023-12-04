#include "i2c.h"

void I2C_Init(void) {
    // Enable I2C1 peripheral clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    // Configure I2C1 GPIO pins (PA9 and PA10)
    GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);  // Clear mode bits
    GPIOA->MODER |= (2 << GPIO_MODER_MODE9_Pos) | (2 << GPIO_MODER_MODE10_Pos); // Set alternate function mode
    GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFSEL9_Pos) | (4 << GPIO_AFRH_AFSEL10_Pos); // Set alternate function to AF4 (I2C)

    // Configure I2C1 peripheral
    I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C1
    I2C1->CR1 = 0; // Clear CR1 register
    I2C1->TIMINGR = 0x00707CBB; // Set timing values for 100kHz (adjust according to your requirements)
    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C1
}

void I2C_Write(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
    // Wait until the bus is not busy
    while (I2C1->ISR & I2C_ISR_BUSY);

    // Generate start condition
    I2C1->CR2 = (deviceAddress << 1) | I2C_CR2_START;

    // Wait until start condition is generated
    while (!(I2C1->ISR & I2C_ISR_TXIS));

    // Send register address
    I2C1->TXDR = registerAddress;

    // Wait until address is sent
    while (!(I2C1->ISR & I2C_ISR_TXIS));

    // Send data
    I2C1->TXDR = data;

    // Wait until data is sent
    while (!(I2C1->ISR & I2C_ISR_TXE));

    // Generate stop condition
    I2C1->CR2 = I2C_CR2_STOP;
}

uint8_t I2C_Read(uint8_t deviceAddress, uint8_t registerAddress) {
    uint8_t data;

    // Wait until the bus is not busy
    while (I2C1->ISR & I2C_ISR_BUSY);

    // Generate start condition
    I2C1->CR2 = (deviceAddress << 1) | I2C_CR2_START;

    // Wait until start condition is generated
    while (!(I2C1->ISR & I2C_ISR_TXIS));

    // Send register address
    I2C1->TXDR = registerAddress;

    // Wait until address is sent
    while (!(I2C1->ISR & I2C_ISR_TXIS));

    // Generate repeated start condition
    I2C1->CR2 = (deviceAddress << 1) | I2C_CR2_RD_WRN | I2C_CR2_START;

    // Wait until repeated start condition is generated
    while (!(I2C1->ISR & I2C_ISR_RXNE));

    // Read data
    data = I2C1->RXDR;

    // Generate stop condition
    I2C1->CR2 = I2C_CR2_STOP;

    return data;
}
