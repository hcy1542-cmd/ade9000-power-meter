################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../GP/ADE9000.c \
../GP/ADE9000_SPI.c \
../GP/I2c.c \
../GP/MD_RTU.c \
../GP/easyStm32LL\ v11.3.c \
../GP/history.c \
../GP/i2c_val.c \
../GP/init_test_func.c \
../GP/t.c \
../GP/ti.c \
../GP/uart2.c \
../GP/uart2.comm.c 

OBJS += \
./GP/ADE9000.o \
./GP/ADE9000_SPI.o \
./GP/I2c.o \
./GP/MD_RTU.o \
./GP/easyStm32LL\ v11.3.o \
./GP/history.o \
./GP/i2c_val.o \
./GP/init_test_func.o \
./GP/t.o \
./GP/ti.o \
./GP/uart2.o \
./GP/uart2.comm.o 

C_DEPS += \
./GP/ADE9000.d \
./GP/ADE9000_SPI.d \
./GP/I2c.d \
./GP/MD_RTU.d \
./GP/easyStm32LL\ v11.3.d \
./GP/history.d \
./GP/i2c_val.d \
./GP/init_test_func.d \
./GP/t.d \
./GP/ti.d \
./GP/uart2.d \
./GP/uart2.comm.d 


# Each subdirectory must supply rules for building sources it contributes
GP/%.o GP/%.su GP/%.cyclo: ../GP/%.c GP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Cx -c -I"D:/03. Power_Meter/01. Fimware/GP_PM_ADE2_251124a/GP" -I"D:/03. Power_Meter/01. Fimware/GP_PM_ADE2_251124a/GP/Include" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
GP/easyStm32LL\ v11.3.o: ../GP/easyStm32LL\ v11.3.c GP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Cx -c -I"D:/03. Power_Meter/01. Fimware/GP_PM_ADE2_251124a/GP" -I"D:/03. Power_Meter/01. Fimware/GP_PM_ADE2_251124a/GP/Include" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"GP/easyStm32LL v11.3.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-GP

clean-GP:
	-$(RM) ./GP/ADE9000.cyclo ./GP/ADE9000.d ./GP/ADE9000.o ./GP/ADE9000.su ./GP/ADE9000_SPI.cyclo ./GP/ADE9000_SPI.d ./GP/ADE9000_SPI.o ./GP/ADE9000_SPI.su ./GP/I2c.cyclo ./GP/I2c.d ./GP/I2c.o ./GP/I2c.su ./GP/MD_RTU.cyclo ./GP/MD_RTU.d ./GP/MD_RTU.o ./GP/MD_RTU.su ./GP/easyStm32LL\ v11.3.cyclo ./GP/easyStm32LL\ v11.3.d ./GP/easyStm32LL\ v11.3.o ./GP/easyStm32LL\ v11.3.su ./GP/history.cyclo ./GP/history.d ./GP/history.o ./GP/history.su ./GP/i2c_val.cyclo ./GP/i2c_val.d ./GP/i2c_val.o ./GP/i2c_val.su ./GP/init_test_func.cyclo ./GP/init_test_func.d ./GP/init_test_func.o ./GP/init_test_func.su ./GP/t.cyclo ./GP/t.d ./GP/t.o ./GP/t.su ./GP/ti.cyclo ./GP/ti.d ./GP/ti.o ./GP/ti.su ./GP/uart2.comm.cyclo ./GP/uart2.comm.d ./GP/uart2.comm.o ./GP/uart2.comm.su ./GP/uart2.cyclo ./GP/uart2.d ./GP/uart2.o ./GP/uart2.su

.PHONY: clean-GP

