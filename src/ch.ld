/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * STM32F405xG memory setup.
 */
MEMORY
{
    /* Bootloader in the first 16 kBytes */
    boot  : org = 0x08000000, len = 16k
    /* Settings storage in the two following 16 kByte sectors */
    settings : org = 0x08004000, len = 32k
    /* Main program in flash starting at 0x20000 */
    flash : org = 0x08020000, len = 128k
    /* Backup of the main firmware starting at 0x40000 */
    backup : org = 0x08040000, len = 128k
    ram0  : org = 0x20000000, len = 128k    /* SRAM1 + SRAM2 */
    ram1  : org = 0x20000000, len = 112k    /* SRAM1 */
    ram2  : org = 0x2001C000, len = 16k     /* SRAM2 */
    ram3  : org = 0x00000000, len = 0
    ram4  : org = 0x10000000, len = 64k     /* CCM SRAM */
    ram5  : org = 0x40024000, len = 4k      /* BCKP SRAM */
    ram6  : org = 0x00000000, len = 0
    ram7  : org = 0x00000000, len = 0
}

/* Information for bootloader */
__bootloader_start__ = ORIGIN(boot);
__bootloader_end__   = ORIGIN(boot) + LENGTH(boot);
__settings_start__   = ORIGIN(settings);
__settings_end__     = ORIGIN(settings) + LENGTH(settings);
__mainprogram_start__ = ORIGIN(flash);
__mainprogram_end__   = ORIGIN(flash) + LENGTH(flash);
__backup_start__ = ORIGIN(backup);
__backup_end__   = ORIGIN(backup) + LENGTH(backup);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram4);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

INCLUDE chrules.ld
