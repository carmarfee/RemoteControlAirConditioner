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
uint8_t fs_flag = 0;
void DC_Task(uint8_t iKey)
{
	uint8_t Buffer_DC[1] = {0Xff};
	uint8_t Buffer_DC_Zero[1] = {0x00};
	uint8_t addr;
	uint8_t *buffer;

	switch (iKey)
	{
	case 0x1C: // 1
		DC_Motor_Pin_Low();
		fs_flag = 1; // zheng
		addr = 0x0F;
		buffer = Buffer_DC_Zero;
		break;
	case 0x1B: // 2
		DC_Motor_Pin_Low();
		fs_flag = 1; // zheng
		addr = 0x03;
		buffer = Buffer_DC;
		break;
	case 0x1A: // 3
		DC_Motor_Pin_Low();
		fs_flag = 1; // zheng
		addr = 0x0F;
		buffer = Buffer_DC;
		break;
	case 0x14: // 4
		DC_MOtor_Pin_High();
		fs_flag = 0; // zheng
		addr = 0x0A;
		buffer = Buffer_DC;
		break;
	case 0x13: // 5
		DC_MOtor_Pin_High();
		fs_flag = 0; // zheng
		addr = 0x05;
		buffer = Buffer_DC;
		break;
	case 0x12: // 6
		DC_MOtor_Pin_High();
		fs_flag = 0; // zheng
		addr = 0x03;
		buffer = Buffer_DC;
		break;
	default:
		DC_Motor_Pin_Low();
		fs_flag = 2; // zheng
		addr = 0x00;
		buffer = Buffer_DC_Zero;
		break;
	}
	I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, addr, buffer, 1);
}
