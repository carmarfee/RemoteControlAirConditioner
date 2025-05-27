#include "stm32f4xx_hal.h"

/*******************************************************************************
 * Function Name  : beepOnce
 * Description    : 接口函数 - 蜂鸣器响一次
 * Input          : None
 * Output         : None
 * Return         : None
 * Attention      : None
 *******************************************************************************/
void beepOnce(uint32_t delay)
{
	for (int i = 0; i < delay; i++)
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(20);
	}
}