################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/cr_startup_lpc175x_6x.c \
../src/crc.c \
../src/crp.c \
../src/iap.c \
../src/sec_bootloader.c \
../src/trace.c \
../src/uart.c \
../src/xmodem1k.c 

OBJS += \
./src/cr_startup_lpc175x_6x.o \
./src/crc.o \
./src/crp.o \
./src/iap.o \
./src/sec_bootloader.o \
./src/trace.o \
./src/uart.o \
./src/xmodem1k.o 

C_DEPS += \
./src/cr_startup_lpc175x_6x.d \
./src/crc.d \
./src/crp.d \
./src/iap.d \
./src/sec_bootloader.d \
./src/trace.d \
./src/uart.d \
./src/xmodem1k.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_CMSIS=CMSISv2p00_LPC17xx -D__LPC17XX__ -D__REDLIB__ -I"C:\cppwold\secBootLoader\inc" -I"C:\cppwold\triobase\CMSISv2p00_LPC17xx\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


