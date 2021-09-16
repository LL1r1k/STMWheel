################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
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
../MyLibs/Src/i2ckeypad.cpp \
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
./MyLibs/Src/i2ckeypad.o \
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
./MyLibs/Src/i2ckeypad.d \
./MyLibs/Src/ledEffects.d \
./MyLibs/Src/ws2812.d 


# Each subdirectory must supply rules for building sources it contributes
MyLibs/Src/%.o: ../MyLibs/Src/%.cpp MyLibs/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F103xB -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../MyLibs/Inc -I../USB/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

