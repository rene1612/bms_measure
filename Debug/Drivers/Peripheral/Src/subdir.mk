################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Peripheral/Src/ads131m0x.c 

OBJS += \
./Drivers/Peripheral/Src/ads131m0x.o 

C_DEPS += \
./Drivers/Peripheral/Src/ads131m0x.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Peripheral/Src/%.o Drivers/Peripheral/Src/%.su Drivers/Peripheral/Src/%.cyclo: ../Drivers/Peripheral/Src/%.c Drivers/Peripheral/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/Peripheral/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Peripheral-2f-Src

clean-Drivers-2f-Peripheral-2f-Src:
	-$(RM) ./Drivers/Peripheral/Src/ads131m0x.cyclo ./Drivers/Peripheral/Src/ads131m0x.d ./Drivers/Peripheral/Src/ads131m0x.o ./Drivers/Peripheral/Src/ads131m0x.su

.PHONY: clean-Drivers-2f-Peripheral-2f-Src

