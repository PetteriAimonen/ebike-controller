GDB = arm-none-eabi-gdb
OOCD = openocd
OOCDFLAGS = -f interface/stlink-v2.cfg -f target/stm32f4x.cfg

PROJECT = ebike-controller
BUILDDIR = build
UINCDIR += src u8glib

USE_OPT += -Os -g -gdwarf-2 -g3 -Wno-unused \
        -fomit-frame-pointer -falign-functions=16 -std=gnu99 -ffast-math

# Bootloader
PROJECT_CSRC += src/bootloader.c
USE_OPT += -DCORTEX_VTOR_INIT=0x00004000

# Base system & bootup
PROJECT_CSRC += src/main.c src/board.c src/debug.c

# Shell
PROJECT_CSRC += src/usbcfg.c src/usb_usart.c src/bluetooth_usart.c src/shell_commands.c

# Motor control
PROJECT_CSRC += src/motor_control.c src/motor_orientation.c src/motor_sampling.c src/motor_limits.c

# Sensors
PROJECT_CSRC += src/lsm6ds3.c src/sensor_task.c src/wheel_speed.c

# Filesystem and logging
PROJECT_CSRC += src/filesystem.c src/log_task.c

# Motion control
PROJECT_CSRC += src/bike_control_task.c

# OLED screen
PROJECT_CSRC += src/ui_task.c
PROJECT_CSRC += u8glib/u8g_ll_api.c u8glib/u8g_com_api.c u8glib/u8g_state.c
PROJECT_CSRC += u8glib/u8g_pb.c u8glib/u8g_pb8v1.c u8glib/u8g_com_null.c
PROJECT_CSRC += u8glib/u8g_clip.c
PROJECT_CSRC += u8glib/u8g_sys_chibios.c
PROJECT_CSRC += u8glib/u8g_font.c u8glib/u8g_font_courb18.c u8glib/u8g_font_8x13.c
PROJECT_CSRC += u8glib/u8g_line.c u8glib/u8g_page.c
PROJECT_CSRC += u8glib/u8g_com_i2c_chibios.c u8glib/u8g_dev_ssd1306_128x64.c

UADEFS =
ULIBDIR = src
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
	$(GDB) -iex 'target remote | $(OOCD) -d1 $(OOCDFLAGS) \
		-c "gdb_port pipe"' -iex 'mon halt' -ex 'set *((uint32_t*)0xe0042004) = 0x07' $(BUILDDIR)/$(PROJECT).elf

%.o: %.c
    # Just to make kdevelop include path discovery work.
	$(CC) -c $(CPFLAGS) $(IINCDIR) $< -o $@

# Chibios's build rules want to generate listings, but they are just waste
# of disk space.
all: rmlists
rmlists:
	rm -rf $(BUILDDIR)/lst/*
