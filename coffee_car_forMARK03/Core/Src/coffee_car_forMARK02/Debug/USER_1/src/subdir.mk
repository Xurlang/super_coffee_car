################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER_1/src/ZLAC8030l.c \
../USER_1/src/timer.c 

OBJS += \
./USER_1/src/ZLAC8030l.o \
./USER_1/src/timer.o 

C_DEPS += \
./USER_1/src/ZLAC8030l.d \
./USER_1/src/timer.d 


# Each subdirectory must supply rules for building sources it contributes
USER_1/src/%.o: ../USER_1/src/%.c USER_1/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USER_1-2f-src

clean-USER_1-2f-src:
	-$(RM) ./USER_1/src/ZLAC8030l.d ./USER_1/src/ZLAC8030l.o ./USER_1/src/timer.d ./USER_1/src/timer.o

.PHONY: clean-USER_1-2f-src

