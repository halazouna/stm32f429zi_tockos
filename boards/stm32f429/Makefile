# Makefile for building the tock kernel for the Hail platform
TOCK_ARCH=cortex-m4
TARGET=thumbv7em-none-eabi
PLATFORM=stm32f429zi

include ../Makefile.common

# TOCKLOADER=tockloader

# # Where in the SAM4L flash to load the kernel with `tockloader`
# KERNEL_ADDRESS=0x10000

.PHONY: apps/$(APP)/build/$(TOCK_ARCH)/app.bin
apps/$(APP)/build/$(TOCK_ARCH)/app.bin:
	@make -C apps/$(APP) TOCK_ARCH=$(TOCK_ARCH)

target/$(TARGET)/release/stm32f429zidk-$(APP): target/$(TARGET)/release/stm32f429zidk apps/$(APP)/build/$(TOCK_ARCH)/app.bin
	@$(OBJCOPY) --update-section .apps=../../userland/examples/$(APP)/build/$(TOCK_ARCH)/app.bin \
	  --set-section-flags .apps=alloc,code \
	  target/$(TARGET)/release/stm32f429zidk $@

target/$(TARGET)/release/stm32f429zidk-$(APP).hex: target/$(TARGET)/release/stm32f429zidk-$(APP)
	@$(OBJCOPY) -Oihex $^ $@

JLINK=JLinkExe
JLINK_OPTIONS+=-device stm32f429zi -if swd -speed 1200 -AutoConnect 1
JLINK_SCRIPTS_DIR=jtag/

# Upload the kernel over JTAG
.PHONY: flash
flash: target/$(TARGET)/release/stm32f429zidk.hex
	$(JLINK) $(JLINK_OPTIONS) $(JLINK_SCRIPTS_DIR)/flash-kernel.jlink

# Upload the kernel over serial/bootloader
.PHONY: program
program: target/$(TARGET)/release/stm32f429zidk.hex
	$(error Cannot program nRF52DK over USB. Use \`make flash\` and JTAG)
