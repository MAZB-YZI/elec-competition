################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI_MSPM0/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0" -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0/Debug" -I"D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-720537846: ../empty_mspm0g3507.syscfg
	@echo 'SysConfig - building file: "$<"'
	"D:/TI_MSPM0/sysconfig_1.26.2/sysconfig_cli.bat" -s "D:/TI_MSPM0/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "D:/TI_MSPM0/CCS_Workspace/Liner_Car0/empty_mspm0g3507.syscfg" -o "syscfg" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/device_linker.cmd: build-720537846 ../empty_mspm0g3507.syscfg
syscfg/device.opt: build-720537846
syscfg/device.cmd.genlibs: build-720537846
syscfg/ti_msp_dl_config.c: build-720537846
syscfg/ti_msp_dl_config.h: build-720537846
syscfg/Event.dot: build-720537846
syscfg: build-720537846

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI_MSPM0/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0" -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0/Debug" -I"D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI_MSPM0/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"syscfg/device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0" -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0/Debug" -I"D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI_MSPM0/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"D:/TI_MSPM0/CCS_Workspace/Liner_Car0/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


