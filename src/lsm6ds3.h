#pragma once

#define LSM6DS3_FIFO_CTRL1      0x06
#define LSM6DS3_FIFO_CTRL2      0x07
#define LSM6DS3_FIFO_CTRL3      0x08
#define LSM6DS3_FIFO_CTRL4      0x09
#define LSM6DS3_FIFO_CTRL5      0x0A
#define LSM6DS3_INT1_CTRL       0x0D
#define LSM6DS3_INT2_CTRL       0x0E
#define LSM6DS3_WHOAMI          0x0F
#define LSM6DS3_CTRL1_XL        0x10
#define LSM6DS3_CTRL2_G         0x11
#define LSM6DS3_CTRL3_C         0x12
#define LSM6DS3_CTRL4_C         0x13
#define LSM6DS3_CTRL5_C         0x14
#define LSM6DS3_CTRL6_C         0x15
#define LSM6DS3_CTRL7_G         0x16
#define LSM6DS3_CTRL8_XL        0x17
#define LSM6DS3_CTRL9_XL        0x18
#define LSM6DS3_CTRL10_C        0x19
#define LSM6DS3_STATUS_REG      0x1E
#define LSM6DS3_OUT_TEMP_L      0x20
#define LSM6DS3_OUT_TEMP_H      0x21
#define LSM6DS3_OUTX_L_G        0x22
#define LSM6DS3_OUTX_H_G        0x23
#define LSM6DS3_OUTY_L_G        0x24
#define LSM6DS3_OUTY_H_G        0x25
#define LSM6DS3_OUTZ_L_G        0x26
#define LSM6DS3_OUTZ_H_G        0x27
#define LSM6DS3_OUTX_L_XL       0x28
#define LSM6DS3_OUTX_H_XL       0x29
#define LSM6DS3_OUTY_L_XL       0x2A
#define LSM6DS3_OUTY_H_XL       0x2B
#define LSM6DS3_OUTZ_L_XL       0x2C
#define LSM6DS3_OUTZ_H_XL       0x2D

#include <stdbool.h>

void lsm6ds3_init();
void lsm6ds3_write(uint8_t reg, uint8_t value);
uint8_t lsm6ds3_read(uint8_t reg);
bool lsm6ds3_read_acc(int *x, int *y, int *z);
bool lsm6ds3_read_gyro(int* x, int* y, int* z);
