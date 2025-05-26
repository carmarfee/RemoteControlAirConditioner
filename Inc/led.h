/* 请使用此文件代替zlg7290.h */
#ifndef __led_H
#define __led_H
#include "stm32f4xx_hal.h"

void initLED(void);
uint8_t updateLED_T(uint8_t target);
uint8_t updateLED_A(uint8_t actual);
void DisplayLED_Off(void);
#endif