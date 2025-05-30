#include "Dc_motor.h"

#define I2C_Open_LONG_TIMEOUT ((uint32_t)0xffff)

__IO uint32_t I2CTimeout1 = I2C_Open_LONG_TIMEOUT;

void I2C_DC_Motor_WriteOneByte(I2C_HandleTypeDef *I2Cx, uint8_t I2C_Addr, uint8_t addr, uint8_t value)
{
	while (HAL_I2C_Mem_Write(I2Cx, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &value, 0x01, I2CTimeout1) != HAL_OK)
	{
	};
}

void I2C_DC_Motor_Write(I2C_HandleTypeDef *I2Cx, uint8_t I2C_Addr, uint8_t addr, uint8_t *buf, uint8_t num)
{
	while (num--)
	{
		I2C_DC_Motor_WriteOneByte(I2Cx, I2C_Addr, addr++, *buf++);
		HAL_Delay(5);
	}
}



/* void DC_Task(FanSpeed speedLevel)
{
	uint8_t Buffer_DC[2] = {0X00, 0xff};		// 直流电机速度4档
	uint8_t addr[4] = {0x00, 0x03, 0x05, 0x0A}; // 直流电机速度4档对应寄存器地址

	switch (speedLevel)
	if (speed == 0)
	{
		I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, addr[speed], &Buffer_DC[0], 1);
	}
	else
	{
		I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, addr[speed], &Buffer_DC[1], 1);
	}
} */
