################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002Testapp_SPI_Sl_arduino.c 

OBJS += \
./Src/002Testapp_SPI_Sl_arduino.o 

C_DEPS += \
./Src/002Testapp_SPI_Sl_arduino.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I"C:/Users/mehme/STM32CubeIDE/workspace_1.16.1/stm32f4xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/002Testapp_SPI_Sl_arduino.cyclo ./Src/002Testapp_SPI_Sl_arduino.d ./Src/002Testapp_SPI_Sl_arduino.o ./Src/002Testapp_SPI_Sl_arduino.su

.PHONY: clean-Src

