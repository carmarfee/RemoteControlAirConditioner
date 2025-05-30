/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ZLG7290_H

#define __ZLG7290_H

#include "stm32f4xx_hal.h"

#define ADDR_24LC64     0x70

#define I2C_PAGESIZE    8

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
