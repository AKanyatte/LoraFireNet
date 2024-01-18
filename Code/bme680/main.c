#include "i2c.h"

int main(void) {
    // Initialize the I2C peripheral
    I2C_Init();

    // Write data to a register
    I2C_Write(0x77, 0x10, 0x42);

    // Read data from a register
    uint8_t data = I2C_Read(0x77, 0x20);

    // logic here...

    while (1) {
        // Main loop
    }
}
