################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../GP/src/iwdg.c \
../GP/src/uart.c 

OBJS += \
./GP/src/iwdg.o \
./GP/src/uart.o 

C_DEPS += \
./GP/src/iwdg.d \
./GP/src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
GP/src/%.o GP/src/%.su GP/src/%.cyclo: ../GP/src/%.c GP/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Cx -c -I"D:/03. Power_Meter/01. Fimware/GP_PM_ADE2_251124a/GP" -I"D:/03. Power_Meter/01. Fimware/GP_PM_ADE2_251124a/GP/Include" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-GP-2f-src

clean-GP-2f-src:
	-$(RM) ./GP/src/iwdg.cyclo ./GP/src/iwdg.d ./GP/src/iwdg.o ./GP/src/iwdg.su ./GP/src/uart.cyclo ./GP/src/uart.d ./GP/src/uart.o ./GP/src/uart.su

.PHONY: clean-GP-2f-src

