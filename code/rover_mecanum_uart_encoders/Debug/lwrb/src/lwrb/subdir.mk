################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lwrb/src/lwrb/lwrb.c \
../lwrb/src/lwrb/lwrb_ex.c 

OBJS += \
./lwrb/src/lwrb/lwrb.o \
./lwrb/src/lwrb/lwrb_ex.o 

C_DEPS += \
./lwrb/src/lwrb/lwrb.d \
./lwrb/src/lwrb/lwrb_ex.d 


# Each subdirectory must supply rules for building sources it contributes
lwrb/src/lwrb/%.o lwrb/src/lwrb/%.su lwrb/src/lwrb/%.cyclo: ../lwrb/src/lwrb/%.c lwrb/src/lwrb/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tommaso/git_repos/nucleo_f411re_mecanum/rover_mecanum_uart_encoders/mecanum" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/tommaso/git_repos/nucleo_f411re_mecanum/rover_mecanum_uart_encoders/cJSON" -I"/home/tommaso/git_repos/nucleo_f411re_mecanum/rover_mecanum_uart_encoders/lwpkt/src/include" -I"/home/tommaso/git_repos/nucleo_f411re_mecanum/rover_mecanum_uart_encoders/lwrb/src/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lwrb-2f-src-2f-lwrb

clean-lwrb-2f-src-2f-lwrb:
	-$(RM) ./lwrb/src/lwrb/lwrb.cyclo ./lwrb/src/lwrb/lwrb.d ./lwrb/src/lwrb/lwrb.o ./lwrb/src/lwrb/lwrb.su ./lwrb/src/lwrb/lwrb_ex.cyclo ./lwrb/src/lwrb/lwrb_ex.d ./lwrb/src/lwrb/lwrb_ex.o ./lwrb/src/lwrb/lwrb_ex.su

.PHONY: clean-lwrb-2f-src-2f-lwrb

