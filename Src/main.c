/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
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
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "zlg7290.h"
#include "DC_motor.h"
#include "LM75A.h"
#include "beep.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define ZLG7290MaxIdleTicks 60000 /* ZLG7290按键无操作最大时间阈值 */
#define BeepDelay 40 /* 蜂鸣器延时 */

typedef enum {
  SPEED_LEVEL_0, /* 风扇停转 */
  SPEED_LEVEL_1, /* 档位1 */
  SPEED_LEVEL_2, /* 档位2 */
  SPEED_LEVEL_3, /* 档位3 */
}FanSpeed;

typedef enum {
  STATE_NORMAL, /* 正常模式 */
  STATE_SLEEP, /* 睡眠模式 */
  STATE_POWEROFF, /* 关机模式 */
}SystemState;

uint8_t zlg7290ReadBuffer = 0; /* ZLG7290键值缓冲区 */
uint8_t zlg7290CanRead = 0; /* ZLG7290是否可读标志 */
uint16_t zlg7290IdleTicks = 0; /* ZLG7290按键未被按下时间累加器 */

SystemState currentState = STATE_POWEROFF; /* 通电时工作状态为PowerOff */
FanSpeed currentSpeedLevel = SPEED_LEVEL_1; /* 默认风扇转速为1档 */
FanSpeed lastSpeedLevel = SPEED_LEVEL_0;

uint8_t powerBtnPressed = 0; /* power按钮被按下标志 */
uint8_t anyBtnPressed = 0; /* 任意按钮被按下标志 */

uint8_t actualTemp = 1; /* 实际温度 */
uint8_t targetTemp = 24; /* 目标温度 */

const uint8_t Buffer_DC[2] = {0x00, 0xff};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void handleKey(void);
void handleStateMachine(void);
void handleDCMotor(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  LM75SetMode(CONF_ADDR, NORMOR_MODE); // set LM75A to normal mode
  initLED();                           // initialize LED buffer
  HAL_TIM_Base_Start_IT(&htim3);       // start timer3

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 执行读键值逻辑 */
    if (zlg7290CanRead == 1)
    {
      /* 重置读取标志，读键值 */
      zlg7290CanRead = 0;
      I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &zlg7290ReadBuffer, 1);

      /* 成功读取键值，重置无操作时间，触发蜂鸣器响应 */
      if (currentState == STATE_SLEEP)
        anyBtnPressed = 1;
      if (currentState == STATE_NORMAL)
        zlg7290IdleTicks = 0;
      if (currentState != STATE_POWEROFF)
        beepOnce(BeepDelay);

      /* 根据读缓冲区中的键值，执行不同逻辑 */
      handleKey();

      /* 重置读缓冲区，避免影响下一次读取 */
      zlg7290ReadBuffer = 0;
    }

    /* 执行状态切换逻辑 */
    handleStateMachine();

    /* 延时一段时间，确保actualTemp更新 */
    HAL_Delay(100);

    /* 执行驱动直流电机逻辑 */
    if (currentState != STATE_POWEROFF)
      handleDCMotor();

    /* 延时一段时间，防止总线阻塞 */
    HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 10000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
  uint8_t tmp[1] = {0};
  tmp[0] = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, tmp, 1, 10);
  return ch;
}

/* 系统时钟中断 */
void HAL_SYSTICK_Callback(void)
{
  /* 若数码管当前工作状态为Normal且无操作时间未超过阈值，则累加无操作时间 */
  if (currentState == STATE_NORMAL && zlg7290IdleTicks < ZLG7290MaxIdleTicks)
    zlg7290IdleTicks++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* 接收到EXTI13时，设置zlg7290按键读取标志 */
  if (GPIO_Pin == GPIO_PIN_13)
  {
    zlg7290CanRead = 1;
  }
}

void handleKey(void)
{
  switch (zlg7290ReadBuffer)
  {
    /* 按键A：(Normal | Sleep有效)使targetTemp增加1，上限30 */
    case 0x19:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        if (targetTemp < 30)
          updateLED_T(++targetTemp);
      }
      break;
    /* 按键B：(Normal | Sleep有效)使targetTemp减少1，下限16 */
    case 0x11:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        if (targetTemp > 16)
          updateLED_T(--targetTemp);
      }
      break;
    /* 按键1：(Normal | Sleep有效)设置fanSpeedLevel为SPEED_LEVEL_1 */
    case 0x1C:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        currentSpeedLevel = SPEED_LEVEL_1;
      }
      break;
    /* 按键2：(Normal | Sleep有效)设置fanSpeedLevel为SPEED_LEVEL_2 */
    case 0x1B:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        currentSpeedLevel = SPEED_LEVEL_2;
      }
      break;
    /* 按键3：(Normal | Sleep有效)设置fanSpeedLevel为SPEED_LEVEL_3 */
    case 0x1A:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        currentSpeedLevel = SPEED_LEVEL_3;
      }
      break;
    /* 按键D：(始终有效)power按钮，进行Normal/Sleep和PowerOff之间的状态切换 */
    case 0x01:
      powerBtnPressed = 1;
      break;
    /* 其他按键：无操作 */
    default:
      break;
  }
}

void handleStateMachine(void)
{
  /* 
   * 在Normal和Sleep模式下，存在开机/关机和Normal/Sleep两种状态转换
   * 处理优先级: 开机/关机状态转换 > Normal/Sleep状态转换
   * 并且处理开机/关机后，不再处理Normal/Sleep
   * 在PowerOff模式下，仅存在开机/关机状态转换，仅通过标志位判断是否需要转换开机即可
   */
  switch(currentState)
  {
    case STATE_NORMAL:
      /* 执行关机逻辑 */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;
        /* 熄灭数码管 */
        updateLED(nullBuffer);

        /* 关闭风扇(若本关闭，则不操作) */
        if (lastSpeedLevel != SPEED_LEVEL_0)
        {
          /* 停转 */
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          lastSpeedLevel = SPEED_LEVEL_0;
        }

        currentState = STATE_POWEROFF;
      }
      /* 执行转换到Sleep状态逻辑 */
      else if (zlg7290IdleTicks == ZLG7290MaxIdleTicks)
      {
        zlg7290IdleTicks = 0;
        /* 熄灭8段7位数码管 */
        updateLED(nullBuffer);

        currentState = STATE_SLEEP;
      }
      break;
    case STATE_SLEEP:
      /* 执行关机逻辑 */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;
        /* 关闭风扇(若本关闭，则不操作) */
        if (lastSpeedLevel != SPEED_LEVEL_0)
        {
          /* 停转 */
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          lastSpeedLevel = SPEED_LEVEL_0;
        }

        currentState = STATE_POWEROFF;
      }
      /* 执行转换到Normal状态逻辑 */
      else if (anyBtnPressed == 1)
      {
        anyBtnPressed = 0;
        /* 点亮8段7位数码管 */
        updateLED(LED_Buffer);

        currentState = STATE_NORMAL;
      }
      break;
    case STATE_POWEROFF:
      /* 执行开机逻辑 */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;
        /* 读取出实际温度，将其赋值给actualTemp */
        actualTemp = LM75A_TimerReadTemperature();
        while (actualTemp == 1) {
          actualTemp = LM75A_TimerReadTemperature();
          HAL_Delay(20);
        }
        
        /* (用实际温度更新LED_Buffer后)点亮8段7位数码管 */
        updateLED_A(actualTemp);
        
        /* 蜂鸣器响 */
        beepOnce(BeepDelay);

        currentState = STATE_NORMAL;
      }
      break;
    default:
      /* 不会出现这种情况 */
      break;
  }
}

void handleDCMotor(void)
{
  /* 如果实际温度大于目标温度，则需开启风扇或调整正在运转的风扇转速 */
  if (actualTemp > targetTemp)
  {
    /* 如果上一次风扇转速和这次相同，则无需改变转速 */
    if (lastSpeedLevel != currentSpeedLevel)
    {
      /* 如果不同，改变风扇转速 */
      DC_Motor_Pin_High();
      switch(currentSpeedLevel)
      {
        /* 正向转速1 */
        case SPEED_LEVEL_1:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x0A,&Buffer_DC[1],1);
          break;
        /* 正向转速2 */
        case SPEED_LEVEL_2:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x05,&Buffer_DC[1],1);
          break;
        /* 正向转速3 */
        case SPEED_LEVEL_3:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x03,&Buffer_DC[1],1);
          break;
        default:
          /* 不会出现这种情况 */
          break;
      }
      /* 重置上一次转速为当前转速 */
      lastSpeedLevel = currentSpeedLevel;
    }
  }
  /* 如果实际温度不大于目标温度，则停止正在运转的风扇；若风扇本就停转，则无变化 */
  else
  {
    if (lastSpeedLevel != SPEED_LEVEL_0)
    {
      /* 停转 */
      I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
      lastSpeedLevel = SPEED_LEVEL_0;
    }
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
