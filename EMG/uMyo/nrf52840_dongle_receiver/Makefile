PROJECT_NAME     := nrf52840_dongle_receiver
TARGETS          := nrf52840_dongle_receiver
OUTPUT_DIRECTORY := build/

SDK_ROOT := ../urf_lib/nrf_usdk52
PROJ_DIR := .

# nRF52840 Dongle - 1MB Flash / 256KB RAM
# Bootloader starts at 0xE0000, application at 0x1000 (after MBR)
$(OUTPUT_DIRECTORY)/nrf52840_dongle_receiver.out: \
  LINKER_SCRIPT := $(PROJ_DIR)/nrf52840_dongle.ld

# Source files - Minimal USB CDC test (no radio)
SRC_FILES += \
  $(PROJ_DIR)/usb_cdc_test.c \
  $(PROJ_DIR)/usb_cdc.c \
  $(SDK_ROOT)/gcc/gcc_startup_nrf52.S \
  $(SDK_ROOT)/system_nrf52.c \

# Include folders
INC_FOLDERS += \
  $(PROJ_DIR) \
  $(PROJ_DIR)/../urf_lib \
  $(SDK_ROOT)/device \
  $(SDK_ROOT)/cmsis/include \
  $(SDK_ROOT) \

# Libraries
LIB_FILES += \

# Optimization flags
OPT = -O3

# C flags - Use nRF52832 defines (SDK limitation - no full nRF52840 headers)
# Radio, GPIO, Timer are compatible. USB CDC uses direct register access.
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10059
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52
CFLAGS += -D__STACK_SIZE=0x1000
CFLAGS += -D__HEAP_SIZE=0x400
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags
CXXFLAGS += $(OPT)

# Assembler flags
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10059
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52
ASMFLAGS += -D__STACK_SIZE=0x1000
ASMFLAGS += -D__HEAP_SIZE=0x400

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs

LIB_FILES += -lc -lnosys -lm

.PHONY: default

default: nrf52840_dongle_receiver

TEMPLATE_PATH := $(SDK_ROOT)/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))
