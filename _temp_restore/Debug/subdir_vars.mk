################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../empty_mspm0g3507.syscfg 

C_SRCS += \
../bluetooth.c \
../delay.c \
./syscfg/ti_msp_dl_config.c \
D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../gray_sensor.c \
../main.c \
../motor.c \
../oled.c 

GEN_CMDS += \
./syscfg/device_linker.cmd 

GEN_FILES += \
./syscfg/device_linker.cmd \
./syscfg/device.opt \
./syscfg/ti_msp_dl_config.c 

GEN_MISC_DIRS += \
./syscfg 

C_DEPS += \
./bluetooth.d \
./delay.d \
./syscfg/ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./gray_sensor.d \
./main.d \
./motor.d \
./oled.d 

GEN_OPTS += \
./syscfg/device.opt 

OBJS += \
./bluetooth.o \
./delay.o \
./syscfg/ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./gray_sensor.o \
./main.o \
./motor.o \
./oled.o 

GEN_MISC_FILES += \
./syscfg/device.cmd.genlibs \
./syscfg/ti_msp_dl_config.h \
./syscfg/Event.dot 

GEN_MISC_DIRS__QUOTED += \
"syscfg" 

OBJS__QUOTED += \
"bluetooth.o" \
"delay.o" \
"syscfg\ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"gray_sensor.o" \
"main.o" \
"motor.o" \
"oled.o" 

GEN_MISC_FILES__QUOTED += \
"syscfg\device.cmd.genlibs" \
"syscfg\ti_msp_dl_config.h" \
"syscfg\Event.dot" 

C_DEPS__QUOTED += \
"bluetooth.d" \
"delay.d" \
"syscfg\ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"gray_sensor.d" \
"main.d" \
"motor.d" \
"oled.d" 

GEN_FILES__QUOTED += \
"syscfg\device_linker.cmd" \
"syscfg\device.opt" \
"syscfg\ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../bluetooth.c" \
"../delay.c" \
"./syscfg/ti_msp_dl_config.c" \
"D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../gray_sensor.c" \
"../main.c" \
"../motor.c" \
"../oled.c" 

SYSCFG_SRCS__QUOTED += \
"../empty_mspm0g3507.syscfg" 


