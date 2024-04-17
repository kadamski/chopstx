# Chopstx make rules.

ifeq ($(EMULATION),)
CSRC += $(CHOPSTX)/entry-$(ARCH).c
else
CSRC += $(CHOPSTX)/entry-gnu-linux.c
endif

CSRC += $(CHOPSTX)/chopstx.c

ifneq ($(USE_EVENTFLAG),)
CSRC += $(CHOPSTX)/eventflag.c
endif

ifeq ($(EMULATION),)
CSRC += $(CHOPSTX)/mcu/chx-$(CHIP).c
else
CSRC += $(CHOPSTX)/mcu/chx-gnu-linux.c
endif

ifneq ($(USE_SYS),)
DEFS += -DUSE_SYS
CSRC += $(CHOPSTX)/mcu/sys-$(CHIP).c
endif
ifneq ($(USE_USB),)
ifeq ($(EMULATION),)
CSRC += $(CHOPSTX)/mcu/usb-$(CHIP).c
else
CSRC += $(CHOPSTX)/mcu/usb-usbip.c
endif
endif
ifneq ($(USE_ADC),)
CSRC += $(CHOPSTX)/contrib/adc-$(CHIP).c
endif
ifneq ($(USE_USART),)
CSRC += $(CHOPSTX)/contrib/usart-$(CHIP).c
endif
ifneq ($(USE_ACKBTN),)
CSRC += $(CHOPSTX)/contrib/ackbtn-$(CHIP).c
endif

INCDIR += $(CHOPSTX)

BUILDDIR = build
ifeq ($(EMULATION),)
OUTFILES = $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).bin
ifneq ($(ENABLE_OUTPUT_HEX),)
OUTFILES += $(BUILDDIR)/$(PROJECT).hex
endif
else
OUTFILES  = $(BUILDDIR)/$(PROJECT)
endif


OPT += -ffunction-sections -fdata-sections -fno-common

OBJS    = $(addprefix $(BUILDDIR)/, $(notdir $(CSRC:.c=.o)))

IINCDIR   = $(patsubst %,-I%,$(INCDIR))
LLIBDIR   = $(patsubst %,-L%,$(LIBDIR))

VPATH     = $(sort $(dir $(CSRC)))
###
ifeq ($(EMULATION),)
SPECS = --specs=picolibc.specs
LDFLAGS   = $(MCFLAGS) -nostartfiles -T$(LDSCRIPT) \
    -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch,--gc-sections
ifeq ($(ARCH),riscv32)
MCFLAGS   = -march=rv32imac_zicsr -mabi=ilp32
LDFLAGS   += -march=rv32imac -mabi=ilp32 # Override arch selection in MCFLAGS
else
MCFLAGS   = -mcpu=$(MCU) -masm-syntax-unified
endif
else
SPECS =
MCFLAGS   =
LDFLAGS   =
DEFS      += -D_GNU_SOURCE
endif
DEFS      += -DARCH_HEADER='"chopstx-$(ARCH).h"' -DARCH_IMPL='"chopstx-$(ARCH).c"'

CFLAGS    = $(MCFLAGS) $(OPT) $(CWARN) -Wa,-alms=$(BUILDDIR)/$(notdir $(<:.c=.lst)) $(DEFS)
LDFLAGS  += $(LLIBDIR)

ifeq ($(EMULATION),)
ifeq ($(ARCH),riscv32)
else
CFLAGS   += -mthumb -mno-thumb-interwork -DTHUMB
LDFLAGS  += -mthumb -mno-thumb-interwork
endif
endif

CFLAGS   += -MD -MP -MF .dep/$(@F).d

all: $(OUTFILES)

$(OBJS): | $(BUILDDIR)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(OBJS) : $(BUILDDIR)/%.o : %.c Makefile
	@echo
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

ifeq ($(EMULATION),)
%.elf: $(OBJS) $(OBJS_ADD) $(LDSCRIPT)
	@echo
	$(LD) $(OBJS) $(OBJS_ADD) $(LDFLAGS) $(LIBS) -o $@

%.bin: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O ihex $< $@
else
$(BUILDDIR)/$(PROJECT): $(OBJS) $(OBJS_ADD)
	@echo
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(OBJS_ADD) $(LIBS)
endif

clean:
	-rm -f -r .dep $(BUILDDIR)

# Include dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
