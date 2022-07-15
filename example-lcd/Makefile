# Makefile for example application of Chopstx

PROJECT = sample

### Currently, it is for GD32VF103
###
### Please change lines started with '###' for STM32 Nucleo L432.
###
### Please change lines started with '###' for Cortex-M3 board.
###
### Please change lines started with '###' for Cortex-M0+ board
### (STM32F0 Discovery).

CHOPSTX = ..
LDSCRIPT= sample.ld.gd32vf103
### LDSCRIPT= sample.ld.m4
### LDSCRIPT= sample.ld
### LDSCRIPT= sample.ld.m3
CSRC = sample.c usb-vendor-specific.c
# CSRC += debug.c

ARCH=riscv32
CHIP=gd32vf103
### CHIP=stm32l4
### USE_SYS = yes
USE_USART = yes
USE_USB = yes

###################################
### CROSS = arm-none-eabi-
CROSS = riscv64-unknown-elf-
CC   = $(CROSS)gcc $(SPECS)
LD   = $(CROSS)gcc $(SPECS)
OBJCOPY   = $(CROSS)objcopy

### MCU   = cortex-m4
### MCU   = cortex-m3
### MCU   = cortex-m0
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DFREE_STANDING -DMHZ=96
### DEFS  = -DUSE_SYS3 -DFREE_STANDING -DMHZ=80
### DEFS  = -DFREE_STANDING -DUSE_SYS3 -DBUSY_LOOP -DCHX_FLAGS_MAIN=CHOPSTX_SCHED_RR
OPT   = -O3 -Os -g
LIBS  =
#LIBS  = -lgcc

CSRC += $(CHOPSTX)/contrib/spi-st.c

####################
include ../rules.mk

board.h:
	@echo Please make a symbolic link \'board.h\' to a file in ../board;
	@exit 1

sys.c: board.h

distclean: clean
	rm -f board.h
