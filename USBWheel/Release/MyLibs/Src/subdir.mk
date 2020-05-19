################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MyLibs/Src/AdcHandler.cpp \
../MyLibs/Src/CmdParser.cpp \
../MyLibs/Src/CommandHandler.cpp \
../MyLibs/Src/EncoderLocal.cpp \
../MyLibs/Src/ExtiHandler.cpp \
../MyLibs/Src/FFBWheel.cpp \
../MyLibs/Src/FFBWheel_commands.cpp \
../MyLibs/Src/FFBWheel_usb_init.cpp \
../MyLibs/Src/Filters.cpp \
../MyLibs/Src/HidFFB.cpp \
../MyLibs/Src/LocalButtons.cpp \
../MyLibs/Src/MotorBTS7960.cpp \
../MyLibs/Src/TimerHandler.cpp \
../MyLibs/Src/UsbHidHandler.cpp \
../MyLibs/Src/global_callbacks.cpp \
../MyLibs/Src/ledEffects.cpp \
../MyLibs/Src/ws2812.cpp 

OBJS += \
./MyLibs/Src/AdcHandler.o \
./MyLibs/Src/CmdParser.o \
./MyLibs/Src/CommandHandler.o \
./MyLibs/Src/EncoderLocal.o \
./MyLibs/Src/ExtiHandler.o \
./MyLibs/Src/FFBWheel.o \
./MyLibs/Src/FFBWheel_commands.o \
./MyLibs/Src/FFBWheel_usb_init.o \
./MyLibs/Src/Filters.o \
./MyLibs/Src/HidFFB.o \
./MyLibs/Src/LocalButtons.o \
./MyLibs/Src/MotorBTS7960.o \
./MyLibs/Src/TimerHandler.o \
./MyLibs/Src/UsbHidHandler.o \
./MyLibs/Src/global_callbacks.o \
./MyLibs/Src/ledEffects.o \
./MyLibs/Src/ws2812.o 

CPP_DEPS += \
./MyLibs/Src/AdcHandler.d \
./MyLibs/Src/CmdParser.d \
./MyLibs/Src/CommandHandler.d \
./MyLibs/Src/EncoderLocal.d \
./MyLibs/Src/ExtiHandler.d \
./MyLibs/Src/FFBWheel.d \
./MyLibs/Src/FFBWheel_commands.d \
./MyLibs/Src/FFBWheel_usb_init.d \
./MyLibs/Src/Filters.d \
./MyLibs/Src/HidFFB.d \
./MyLibs/Src/LocalButtons.d \
./MyLibs/Src/MotorBTS7960.d \
./MyLibs/Src/TimerHandler.d \
./MyLibs/Src/UsbHidHandler.d \
./MyLibs/Src/global_callbacks.d \
./MyLibs/Src/ledEffects.d \
./MyLibs/Src/ws2812.d 


# Each subdirectory must supply rules for building sources it contributes
MyLibs/Src/AdcHandler.o: ../MyLibs/Src/AdcHandler.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/AdcHandler.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/CmdParser.o: ../MyLibs/Src/CmdParser.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/CmdParser.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/CommandHandler.o: ../MyLibs/Src/CommandHandler.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/CommandHandler.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/EncoderLocal.o: ../MyLibs/Src/EncoderLocal.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/EncoderLocal.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/ExtiHandler.o: ../MyLibs/Src/ExtiHandler.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/ExtiHandler.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/FFBWheel.o: ../MyLibs/Src/FFBWheel.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/FFBWheel.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/FFBWheel_commands.o: ../MyLibs/Src/FFBWheel_commands.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/FFBWheel_commands.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/FFBWheel_usb_init.o: ../MyLibs/Src/FFBWheel_usb_init.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/FFBWheel_usb_init.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/Filters.o: ../MyLibs/Src/Filters.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/Filters.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/HidFFB.o: ../MyLibs/Src/HidFFB.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/HidFFB.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/LocalButtons.o: ../MyLibs/Src/LocalButtons.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/LocalButtons.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/MotorBTS7960.o: ../MyLibs/Src/MotorBTS7960.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/MotorBTS7960.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/TimerHandler.o: ../MyLibs/Src/TimerHandler.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/TimerHandler.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/UsbHidHandler.o: ../MyLibs/Src/UsbHidHandler.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/UsbHidHandler.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/global_callbacks.o: ../MyLibs/Src/global_callbacks.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/global_callbacks.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/ledEffects.o: ../MyLibs/Src/ledEffects.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/ledEffects.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyLibs/Src/ws2812.o: ../MyLibs/Src/ws2812.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB/Inc -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"MyLibs/Src/ws2812.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

