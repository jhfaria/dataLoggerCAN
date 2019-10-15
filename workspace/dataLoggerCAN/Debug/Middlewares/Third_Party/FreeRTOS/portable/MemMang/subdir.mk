################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/jhfaria/Documents/FreeRTOS/STM32F4/portable/MemMang/heap_2.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/portable/MemMang/heap_2.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/portable/MemMang/heap_2.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/portable/MemMang/heap_2.o: /home/jhfaria/Documents/FreeRTOS/STM32F4/portable/MemMang/heap_2.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F429xx -c -I../Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I/home/jhfaria/Documents/FreeRTOS/STM32F4/include -I/home/jhfaria/Documents/FreeRTOS/STM32F4/portable/GCC/ARM_CM4F -I/home/jhfaria/Documents/FreeRTOS/STM32F4/config -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/portable/MemMang/heap_2.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

