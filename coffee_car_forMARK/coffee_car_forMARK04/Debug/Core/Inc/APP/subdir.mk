################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/APP/motor_can.c 

OBJS += \
./Core/Inc/APP/motor_can.o 

C_DEPS += \
./Core/Inc/APP/motor_can.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/APP/%.o Core/Inc/APP/%.su Core/Inc/APP/%.cyclo: ../Core/Inc/APP/%.c Core/Inc/APP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-APP

clean-Core-2f-Inc-2f-APP:
	-$(RM) ./Core/Inc/APP/motor_can.cyclo ./Core/Inc/APP/motor_can.d ./Core/Inc/APP/motor_can.o ./Core/Inc/APP/motor_can.su

.PHONY: clean-Core-2f-Inc-2f-APP

