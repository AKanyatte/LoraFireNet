################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BME680_driver-master/bme680.c 

OBJS += \
./Core/BME680_driver-master/bme680.o 

C_DEPS += \
./Core/BME680_driver-master/bme680.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BME680_driver-master/%.o Core/BME680_driver-master/%.su Core/BME680_driver-master/%.cyclo: ../Core/BME680_driver-master/%.c Core/BME680_driver-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/salma/OneDrive/Desktop/Capstone/Code/BME680_driver-master/BME680_driver-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BME680_driver-2d-master

clean-Core-2f-BME680_driver-2d-master:
	-$(RM) ./Core/BME680_driver-master/bme680.cyclo ./Core/BME680_driver-master/bme680.d ./Core/BME680_driver-master/bme680.o ./Core/BME680_driver-master/bme680.su

.PHONY: clean-Core-2f-BME680_driver-2d-master

