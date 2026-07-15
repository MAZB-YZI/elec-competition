# MSPM0G3507 项目 Makefile
# 使用 TI ARM Clang 编译器

# 工具链路径（如果环境变量未设置，使用默认值）
TI_ARM_CGT_ROOT ?= D:/TI/Tools/ti_cgt_arm_llvm_4.0.2.LTS
MSPM0_SDK_ROOT ?= D:/TI/SDK/mspm0_sdk_2_10_00_04
SYSCONFIG_ROOT ?= D:/TI/Tools/sysconfig_1.26.2

CC = "$(TI_ARM_CGT_ROOT)/bin/tiarmclang"
LNK = "$(TI_ARM_CGT_ROOT)/bin/tiarmclang"

# SDK 路径
SDK_DIR = $(subst \,/,$(MSPM0_SDK_ROOT))

# SysConfig
SYSCONFIG = "$(SYSCONFIG_ROOT)/sysconfig_cli.bat"
SYSCFG_CMD = $(SYSCONFIG) --compiler ticlang --product "$(SDK_DIR)/.metadata/product.json"

# 项目根目录（从 makefile 位置推算）
PROJECT_ROOT = $(subst \,/,$(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..))

# 项目文件
PROJECT_NAME = empty
SYSCFG_FILE = $(PROJECT_ROOT)/empty.syscfg
SOURCE_FILE = $(PROJECT_ROOT)/empty.c
STARTUP_FILE = $(SDK_DIR)/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c

# 输出目录
BUILD_DIR = $(PROJECT_ROOT)/build
SYSCFG_DIR = $(PROJECT_ROOT)/syscfg

# 设备宏定义
DEVICE_DEFS = -D__MSPM0G3507__ -D__USE_SYSCONFIG__

# 编译选项
CFLAGS = -mcpu=cortex-m0plus \
    -march=thumbv6m \
    -mfloat-abi=soft \
    -mthumb \
    -O2 \
    -gdwarf-3 \
    -Wall \
    $(DEVICE_DEFS) \
    -I$(PROJECT_ROOT) \
    -I$(SYSCFG_DIR) \
    -I$(SDK_DIR)/source \
    -I$(SDK_DIR)/source/third_party/CMSIS/Core/Include

# 链接选项
LFLAGS = -ldevice.cmd.genlibs \
    -L$(SDK_DIR)/source \
    -L$(SYSCFG_DIR) \
    $(SYSCFG_DIR)/device_linker.cmd \
    -Wl,-m,$(PROJECT_NAME).map \
    -Wl,--rom_model \
    -Wl,--warn_sections \
    -L"$(TI_ARM_CGT_ROOT)/lib" \
    -llibc.a

# 目标文件
OBJECTS = $(BUILD_DIR)/$(PROJECT_NAME).obj $(BUILD_DIR)/ti_msp_dl_config.obj $(BUILD_DIR)/startup_mspm0g350x_ticlang.obj

# 默认目标
all: syscfg $(BUILD_DIR)/$(PROJECT_NAME).out

# 创建目录
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# SysConfig 代码生成
syscfg:
	@echo "=== 运行 SysConfig 生成代码 ==="
	$(SYSCFG_CMD) --output $(SYSCFG_DIR) $(SYSCFG_FILE)

# 编译主文件
$(BUILD_DIR)/$(PROJECT_NAME).obj: $(SOURCE_FILE) $(SYSCFG_DIR)/ti_msp_dl_config.h | $(BUILD_DIR)
	@echo "=== 编译 $(PROJECT_NAME).c ==="
	$(CC) $(CFLAGS) -c $< -o $@

# 编译 SysConfig 生成的配置文件
$(BUILD_DIR)/ti_msp_dl_config.obj: $(SYSCFG_DIR)/ti_msp_dl_config.c $(SYSCFG_DIR)/ti_msp_dl_config.h | $(BUILD_DIR)
	@echo "=== 编译 ti_msp_dl_config.c ==="
	$(CC) $(CFLAGS) -c $< -o $@

# 编译启动文件
$(BUILD_DIR)/startup_mspm0g350x_ticlang.obj: $(STARTUP_FILE) | $(BUILD_DIR)
	@echo "=== 编译 startup_mspm0g350x_ticlang.c ==="
	$(CC) $(CFLAGS) -c $< -o $@

# 链接
$(BUILD_DIR)/$(PROJECT_NAME).out: $(OBJECTS)
	@echo "=== 链接 $(PROJECT_NAME).out ==="
	$(LNK) -Wl,-u,_c_int00 $(OBJECTS) $(LFLAGS) -o $@
	@echo "=== 编译完成: $@ ==="

# 清理
clean:
	@echo "=== 清理编译产物 ==="
	$(RM) -r $(BUILD_DIR)
	$(RM) -r $(SYSCFG_DIR)
	$(RM) $(PROJECT_ROOT)/$(PROJECT_NAME).map

# 重新编译
rebuild: clean all

.PHONY: all clean rebuild syscfg
