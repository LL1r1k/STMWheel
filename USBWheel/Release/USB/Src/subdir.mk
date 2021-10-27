################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB/Src/usb_device.c \
../USB/Src/usbd_cdc.c \
../USB/Src/usbd_cdc_if.c \
../USB/Src/usbd_composite.c \
../USB/Src/usbd_conf.c \
../USB/Src/usbd_core.c \
../USB/Src/usbd_ctlreq.c \
../USB/Src/usbd_custom_hid_if.c \
../USB/Src/usbd_customhid.c \
../USB/Src/usbd_desc.c \
../USB/Src/usbd_ioreq.c 

C_DEPS += \
./USB/Src/usb_device.d \
./USB/Src/usbd_cdc.d \
./USB/Src/usbd_cdc_if.d \
./USB/Src/usbd_composite.d \
./USB/Src/usbd_conf.d \
./USB/Src/usbd_core.d \
./USB/Src/usbd_ctlreq.d \
./USB/Src/usbd_custom_hid_if.d \
./USB/Src/usbd_customhid.d \
./USB/Src/usbd_desc.d \
./USB/Src/usbd_ioreq.d 

OBJS += \
./USB/Src/usb_device.o \
./USB/Src/usbd_cdc.o \
./USB/Src/usbd_cdc_if.o \
./USB/Src/usbd_composite.o \
./USB/Src/usbd_conf.o \
./USB/Src/usbd_core.o \
./USB/Src/usbd_ctlreq.o \
./USB/Src/usbd_custom_hid_if.o \
./USB/Src/usbd_customhid.o \
./USB/Src/usbd_desc.o \
./USB/Src/usbd_ioreq.o 


# Each subdirectory must supply rules for building sources it contributes
USB/Src/%.o: ../USB/Src/%.c USB/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

