PROJECT_NAME     := nrf52840_dk_receiver
TARGETS          := nrf52840_dk_receiver
OUTPUT_DIRECTORY := build/

SDK_ROOT := ../../uMyo/urf_lib/nrf_usdk52
URF_LIB  := ../../uMyo/urf_lib
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/nrf52840_dk_receiver.out: \
  LINKER_SCRIPT := $(PROJ_DIR)/nrf52840_dk.ld

# Source files
SRC_FILES += \
  $(PROJ_DIR)/main.c \
  $(URF_LIB)/urf_radio.c \
  $(URF_LIB)/urf_timer.c \
  $(URF_LIB)/urf_star_protocol.c \
  $(URF_LIB)/urf_uart.c \
  $(SDK_ROOT)/gcc/gcc_startup_nrf52.S \
  $(SDK_ROOT)/system_nrf52.c \

# Include folders
INC_FOLDERS += \
  $(PROJ_DIR) \
  $(URF_LIB) \
  $(SDK_ROOT)/device \
  $(SDK_ROOT)/cmsis/include \
  $(SDK_ROOT) \

# Optimization flags
OPT = -O3

# C flags - Use nRF52832 defines (compatible with nRF52840 for radio/peripherals)
CFLAGS += $(OPT)
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52
CFLAGS += -D__STACK_SIZE=0x2000
CFLAGS += -D__HEAP_SIZE=0x1000
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# Assembler flags
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52
ASMFLAGS += -D__STACK_SIZE=0x2000
ASMFLAGS += -D__HEAP_SIZE=0x1000

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs

LIB_FILES += -lc -lnosys -lm

.PHONY: default flash clean

default: nrf52840_dk_receiver

TEMPLATE_PATH := $(SDK_ROOT)/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

flash: default
	JLinkExe -device NRF52840_XXAA -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash.jlink

clean:
	rm -rf build/
