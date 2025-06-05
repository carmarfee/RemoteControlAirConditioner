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

/* USER CODE BEGIN 0 */
#include "mxconstants.h"
#include "led.h"
#include "key.h"
#include "LM75A.h"
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
static uint16_t tim3Counter = 0;
static uint16_t tim3keyCounter = 0;

/*
 * ��Normal����ģʽ�£�countΪ400�ı���ʱ��ȡһ���¶�
 * ��Sleep�£�countΪ2000�ı���ʱ��ȡһ���¶�
 * PowerOff����ģʽ����ȡ�¶�
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3)
    return;

  /* ��ǰ����ģʽΪPowerOffʱ��ִ�к����߼� */
  if (systemState.currentState == STATE_POWEROFF && systemState.tempReadCnt == TEMPBUFFER_SIZE - 1)
    return;

  /************************************** 温度采样滤波 **************************************/

  tim3Counter++;

  /* ����ģʽѡ��ͬ�Ĳ���Ƶ�� */
  uint16_t sampleInterval = (systemState.currentState == STATE_SLEEP) ? 60 : 20;

  if (tim3Counter % sampleInterval == 0)
  {
    uint8_t newTemp = readActualTemp();

    /* ��ȡ����һ����Ч�¶��Һ͵�ǰ�¶Ȳ�ͬʱ���滻��ǰ�¶� */
    if (newTemp != 1 && newTemp)
    {
      systemState.actualTemp = newTemp;
      if (systemState.currentState == STATE_NORMAL)
        updateLED();
    }
  }
  /************************************** 键盘输入消抖 **************************************/
  uint8_t keyIntervval = 600; // 10ms,todo:如何保证10ms
  // 大概10ms一次需要发生一次中断进行按键扫描
  if (tim3Counter % keyIntervval && is_pressed)
  {
    keyScan();
    for (uint8_t i = 0; i < 3; i++)
    {
      switch (key[i].keyState)
      {
      case KEY_IDLE:
        if (key[i].judgeTimes == 1) // 只判断了一次，第二次检测未pressed,判定为抖动
          key[i].judgeTimes = 0;
        if (key[i].judgeTimes > 2) // 判断两次后进入idle,说明按键释放,为短按
          key[i].judgeTimes = 0;
        break;
      case KEY_PRESSED:
        if (key[i].judgeTimes == 0)
          key[i].judgeTimes = 1; // 判断一次,进入待确认状态
        else if (key[i].judgeTimes == 1)
        {
          key[i].judgeTimes = 2; // 判断多次,进入确认状态
          key[i].keyState = KEY_CONFIRMED;
        }
        else // 判断次数未清零，说明一直未释放，为长按.
        {
          key[i].judgeTimes++; // 判断多次,进入确认状态
          key[i].keyState = KEY_CONFIRMED;
        }
        break;
      case KEY_CONFIRMED:
        systemState.zlg7290KeyStates.canRead = 1;
        systemState.zlg7290KeyStates.readBuffer = key[i].keyCode;
        break;
      default:
        Error_Handler();
        break;
      }
    }
  }
  if (tim3Counter >= 60000)
    tim3Counter = 0;
}

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
