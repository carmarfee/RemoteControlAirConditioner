/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ZLG7290_H

#define __ZLG7290_H

#include "stm32f4xx_hal.h"

#define ADDR_24LC64     0x70

#define I2C_PAGESIZE    8

typedef enum {
    Normal,
    Sleep
}ZLG7290State;

typedef struct ZLG7290DataHandler {
  uint8_t readBuffer; // ZLG7290键值缓冲区
  uint8_t canRead; // ZLG7290是否可读
  uint16_t idleTicks; // ZLG7290按键“未被按下”的时间
  ZLG7290State state; // ZLG7290当前工作状态
  uint8_t needSwitchState; // ZLG7290是否需要切换状态
}zlg7290h;


void I2C_ZLG7290_Read(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num);
void I2C_ZLG7290_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num);

void initLED(void);
uint8_t updateLED_T(uint8_t target);
uint8_t updateLED_A(uint8_t actual);
void updateLED(uint8_t *buffer);

extern uint8_t LED_Buffer[8];
extern uint8_t nullBuffer[8];

#endif /* __24C64_OPT_H */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
