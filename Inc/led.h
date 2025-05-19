/* 请使用此文件代替zlg7290.h */
#ifndef __led_H

#define __led_H

#include "stm32f4xx_hal.h"

#define ADDR_24LC64     0x70

#define I2C_PAGESIZE    8

void initLED();
uint8_t updateLED_T(uint8_t target);
uint8_t updateLED_A(uint8_t actual);

#endif