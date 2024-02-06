################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/stm32-hal-rfm95-master/rfm95.c 

OBJS += \
./Core/stm32-hal-rfm95-master/rfm95.o 

C_DEPS += \
./Core/stm32-hal-rfm95-master/rfm95.d 


# Each subdirectory must supply rules for building sources it contributes
Core/stm32-hal-rfm95-master/%.o Core/stm32-hal-rfm95-master/%.su Core/stm32-hal-rfm95-master/%.cyclo: ../Core/stm32-hal-rfm95-master/%.c Core/stm32-hal-rfm95-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/salma/OneDrive/Desktop/Capstone/Code/stm32-hal-rfm95-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-stm32-2d-hal-2d-rfm95-2d-master

clean-Core-2f-stm32-2d-hal-2d-rfm95-2d-master:
	-$(RM) ./Core/stm32-hal-rfm95-master/rfm95.cyclo ./Core/stm32-hal-rfm95-master/rfm95.d ./Core/stm32-hal-rfm95-master/rfm95.o ./Core/stm32-hal-rfm95-master/rfm95.su

.PHONY: clean-Core-2f-stm32-2d-hal-2d-rfm95-2d-master

