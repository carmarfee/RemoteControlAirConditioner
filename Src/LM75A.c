#include "LM75A.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"

volatile uint16_t LM75A_Temperature = 0; // �洢���¶�ȡ���¶�
volatile uint8_t LM75A_ReadReady = 0;	 // ��־λ��ָʾ�Ƿ����µ��¶����ݿ���

/**
 * @brief 甚至温度传感器的工作模式
 *
 * @param ConfReg 配置寄存器
 * @param Mode 工作状态字节
 * @return uint8_t 返回是否配置成功
 */
uint8_t LM75SetMode(uint8_t ConfReg, uint8_t Mode)
{
	uint8_t Config;
	if (HAL_I2C_Mem_Write(&hi2c1, 0x9F, ConfReg, 1, &Mode, 1, 100) == HAL_OK)
	{
		if ((HAL_I2C_Mem_Read(&hi2c1, 0x9F, ConfReg, 1, &Config, 1, 100) == HAL_OK) && ((Config && Mode) == Mode))
		{
#if DEBUG
			printf("current conf_reg: %02x\n", Config);
#endif
			return EVL_OK;
		}
	}
	return EVL_ER;
}

/**
 * @brief 读取 LM75A 温度传感器的温度寄存器值
 *
 * @return uint16_t 是否读取成功。如果成功，则返回温度寄存器的值；否则返回 EVL_ER。
 */
uint16_t LM75GetTempReg(void)
{
	uint8_t tempreg[2];
	uint16_t temp;
	if (HAL_I2C_Mem_Read(&hi2c1, 0x9F, TEMP_ADDR, 2, tempreg, 2, 100) == HAL_OK)
	{
		temp = ((tempreg[0] << 8) | tempreg[1]) >> 5;
#if DEBUG
		printf("current Temp: %04x\n", temp);
#endif
		return temp;
	}
	return EVL_ER;
}

/**
 * @brief 将温度寄存器值转换为实际温度值。
 *
 * @param tempreg 温度寄存器的值。
 * @return double 返回实际温度值
 */

double LM75GetTempValue(uint16_t tempreg)
{
	double TempValue;
	if (tempreg & (0x01 << 16))
	{
		TempValue = ((!tempreg) + 1) * 0.125;
		printf("TempValue: %lf`C\n", TempValue);
	}
	else
	{
		TempValue = tempreg * 0.125;
		printf("TempValue: %lf`C\n", TempValue);
	}
	return TempValue;
}

#define BUFFER_SIZE 10 // Define the size of the buffer for moving average

uint8_t tempBuffer[BUFFER_SIZE] = {0};
uint8_t bufferIndex = 0;
uint16_t tempSum = 0;
/**
 * @brief 通过定时器中断读取温度。定时器回调函数执行的内容。
 *
 */
uint8_t LM75A_TimerReadTemperature(void)
{
	uint8_t actualTemp = getActualTemp();
	if (actualTemp != 1) // Check if the temperature reading is valid
	{
		// Update the buffer with the new temperature reading
		tempSum -= tempBuffer[bufferIndex];
		tempBuffer[bufferIndex] = actualTemp;
		tempSum += tempBuffer[bufferIndex];

		// Move to the next index in the buffer
		bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

		if (bufferIndex >= 9) {
			actualTemp = tempSum / BUFFER_SIZE;
		}
		
		return actualTemp;
	}
	return 1;
}

/*******************************************************************************
 * Function Name  : getActualTemp
 * Description    : 接口函数 - 获取实际温度(整数)
 * Input          : None
 * Output         : None
 * Return         : OK - actualTemp
 *				   Error - 1
 * Attention      : None
 *******************************************************************************/
uint8_t getActualTemp()
{
	uint16_t actualTemp = LM75GetTempReg();
	if (actualTemp == 1)
	{
		return 1;
	}

	if (actualTemp & (0x01 << 16))
	{
		return ((!actualTemp) + 1) >> 3;
	}
	else
	{
		return actualTemp >> 3;
	}
}
