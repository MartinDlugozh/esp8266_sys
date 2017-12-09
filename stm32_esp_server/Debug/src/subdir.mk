################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/UART_freertos.c \
../src/_write.c \
../src/croutine.c \
../src/event_groups.c \
../src/fonts.c \
../src/heap_3.c \
../src/list.c \
../src/main.c \
../src/port.c \
../src/queue.c \
../src/ssd1306.c \
../src/ssd1306_i2c.c \
../src/tasks.c \
../src/timers.c 

OBJS += \
./src/UART_freertos.o \
./src/_write.o \
./src/croutine.o \
./src/event_groups.o \
./src/fonts.o \
./src/heap_3.o \
./src/list.o \
./src/main.o \
./src/port.o \
./src/queue.o \
./src/ssd1306.o \
./src/ssd1306_i2c.o \
./src/tasks.o \
./src/timers.o 

C_DEPS += \
./src/UART_freertos.d \
./src/_write.d \
./src/croutine.d \
./src/event_groups.d \
./src/fonts.d \
./src/heap_3.d \
./src/list.d \
./src/main.d \
./src/port.d \
./src/queue.d \
./src/ssd1306.d \
./src/ssd1306_i2c.d \
./src/tasks.d \
./src/timers.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


