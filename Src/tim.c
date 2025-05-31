/**
 ******************************************************************************
 * File Name          : TIM.c
 * Description        : This file provides code for the configuration
 *                      of the TIM instances.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2025 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "zlg7290.h"

/* USER CODE BEGIN 0 */
extern uint8_t actualTemp;
extern uint8_t targetTemp;
extern zlg7290h zlg7290;
/* USER CODE END 0 */

TIM_HandleTypeDef htim3;

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */
    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */


typedef enum {
  STATE_NORMAL, /* 正常模式 */
  STATE_SLEEP, /* 睡眠模式 */
  STATE_POWEROFF, /* 关机模式 */
}SystemState;

static uint16_t tim3Counter = 0;
extern SystemState currentState;

/* 
 * 在Normal工作模式下，count为400的倍数时读取一次温度
 * 在Sleep工作模式下，count为2000的倍数时读取一次温度
 * 在PowerOff工作模式下，不读取温度
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3)
    return;
  
  /* PowerOff模式不进行任何操作 */
  if (currentState == STATE_POWEROFF)
    return;

  tim3Counter++;  // 计数器递增（Normal/Sleep模式）
  
  /* 根据模式选择不同的采样频率 */
  uint16_t sampleInterval = (currentState == STATE_NORMAL) ? 400 : 2000;
  
  if (tim3Counter % sampleInterval == 0)
  {
    /* 读取温度并更新显示 */
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_15);  // 调试用LED
    
    uint8_t newTemp = LM75A_TimerReadTemperature();
    
    if (newTemp != 1 && currentState == STATE_NORMAL && newTemp != actualTemp)
    {
      actualTemp = newTemp;
      updateLED_A(actualTemp); 
    }
    
    /* 计数器归零防止溢出 */
    if (tim3Counter >= 60000)
      tim3Counter = 0;
  }
}

/* void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    
    if (currentState != STATE_POWEROFF)
    {
      tim3Counter++;

    }
    if (tim3Counter % 400 == 0)
    {
      HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_15);
      uint8_t resultTemp = LM75A_TimerReadTemperature();
      if (resultTemp != 1 && zlg7290.state == (ZLG7290State)Normal)
      {
        actualTemp = resultTemp;
        updateLED_A(actualTemp);
      }
      else{
        updateLED_A(actualTemp);
      }
    }
  }
} */
/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
