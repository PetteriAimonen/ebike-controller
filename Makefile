GDB = arm-none-eabi-gdb
OOCD = openocd
OOCDFLAGS = -f interface/stlink-v2.cfg -f target/stm32f4x.cfg

PROJECT = ebike-controller
BUILDDIR = build
UINCDIR += src

USE_OPT += -Os -g -gdwarf-2 -g3 \
        -fomit-frame-pointer -falign-functions=16 -std=gnu99 -ffast-math

# Base system & bootup
PROJECT_CSRC += src/main.c src/board.c src/debug.c

# Shell
PROJECT_CSRC += src/usbcfg.c src/usb_usart.c src/bluetooth_usart.c src/shell_commands.c

# Motor control
PROJECT_CSRC += src/motor_control.c src/motor_orientation.c src/motor_sampling.c src/motor_limits.c

UADEFS =
ULIBDIR = 
ULIBS = -lm

include Makefile.chibios

program: $(BUILDDIR)/$(PROJECT).elf
	$(OOCD) $(OOCDFLAGS) \
	  -c "init" -c "targets" -c "reset halt" \
	  -c "flash write_image erase $<" -c "verify_image $<" \
	  -c "reset run" -c "shutdown"

debug:
	$(GDB) -iex 'target extended | $(OOCD) -d1 $(OOCDFLAGS) -c "stm32f4x.cpu configure -rtos auto;" \
		-c "gdb_port pipe"' -iex 'mon halt' \
		-ex 'set *((uint32_t*)0xe0042004) = 0x07' $(BUILDDIR)/$(PROJECT).elf

debugraw:
	$(GDB) -iex 'target extended | $(OOCD) -d1 $(OOCDFLAGS) \
		-c "gdb_port pipe"' -iex 'mon halt' -ex 'set *((uint32_t*)0xe0042004) = 0x07' $(BUILDDIR)/$(PROJECT).elf

%.o: %.c
    # Just to make kdevelop include path discovery work.
	$(CC) -c $(CPFLAGS) $(IINCDIR) $< -o $@

# Chibios's build rules want to generate listings, but they are just waste
# of disk space.
all: rmlists
rmlists:
	rm -rf $(BUILDDIR)/lst/*
