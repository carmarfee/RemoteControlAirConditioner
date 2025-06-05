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
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "zlg7290.h"
#include "LM75A.h"
#include "Dc_motor.h"
#include "beep.h"
// #include "check.h"
#include "mxconstants.h"
#include "led.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SystemStateBlock systemState;

// uint32_t unStartFlag __at (0x40003FF4); /* 热启动MagicNumber保存位置 */

uint8_t Buffer_DC[2] = {0x00, 0xff};

volatile uint32_t *unStartFlag = (volatile uint32_t *)0x10000000;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch, FILE *f);
void handleKey(void);
void initSystemState(void);
void handleStateMachine(void);
void handleDCMotor(void);
void handleMarquee(void);
void turnOffMarquee(void);
void initMarquee(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  if (*unStartFlag != MagicNumber) /* 如果是第一次启动 */
  {
    *unStartFlag = MagicNumber;
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
  MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
  initSystemState();
  LM75SetMode(CONF_ADDR, NORMOR_MODE);
  HAL_TIM_Base_Start_IT(&htim3);
  initMarquee();
  }
  else
  {
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
  MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
  initSystemState();
  LM75SetMode(CONF_ADDR, NORMOR_MODE);
  HAL_TIM_Base_Start_IT(&htim3);
  initMarquee();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    /* 执行读键值逻辑 */
    if (systemState.zlg7290KeyStates.canRead == 1)
    {
      /* 重置读取标志，读键值 */
      systemState.zlg7290KeyStates.canRead = 0;
      I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &systemState.zlg7290KeyStates.readBuffer, 1);
     
      if (systemState.currentState == STATE_SLEEP)
        systemState.zlg7290KeyStates.anyBtnPressed = 1;

      /* 成功读取键值，重置无操作时间，触发蜂鸣器响应 */
      if (systemState.currentState == STATE_NORMAL)
        systemState.zlg7290KeyStates.idleTicks = 0;

      if (systemState.currentState != STATE_POWEROFF)
        beepOnce(BeepDelay);

      /* 根据读缓冲区中的键值，执行不同逻辑 */
      handleKey();

      /* 重置读缓冲区，避免影响下一次读取 */
      systemState.zlg7290KeyStates.readBuffer = 0;
    }

    /* 执行状态切换逻辑 */
    handleStateMachine();

    refreshLED();      
    
    /* 执行驱动直流电机逻辑 */
    if (systemState.currentState != STATE_POWEROFF) 
      handleDCMotor();

    /* 更新Marquee */
    if (systemState.currentState == STATE_NORMAL)
      handleMarquee();

    /* 喂狗 */
    MX_IWDG_Refresh();
      
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/10000);

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
  if (systemState.currentState == STATE_NORMAL && systemState.zlg7290KeyStates.idleTicks < ZLG7290MaxIdleTicks)
    systemState.zlg7290KeyStates.idleTicks++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* 接收到EXTI13时，设置zlg7290按键读取标志 */
  if (GPIO_Pin == GPIO_PIN_13)
    systemState.zlg7290KeyStates.canRead = 1;
}

void initSystemState(void)
{
  systemState.zlg7290KeyStates.readBuffer = 0; /* ZLG7290键值缓冲区 */
  systemState.zlg7290KeyStates.canRead = 0; /* ZLG7290是否可读标志 */
  systemState.zlg7290KeyStates.idleTicks = 0; /* ZLG7290按键未被按下时间累加器 */
  systemState.zlg7290KeyStates.powerBtnPressed = 0; /* 通电时工作状态为PowerOff */
  systemState.zlg7290KeyStates.anyBtnPressed = 0; /* 任意按钮被按下标志 */

  systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_1; /* 默认风扇转速为1档 */
  systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;

  systemState.currentMarqueeState = MARQUEE_OFF;
  systemState.currentState = STATE_POWEROFF; /* 通电时工作状态为PowerOff */

  systemState.actualTemp = 1; /* 实际温度 */
  systemState.targetTemp = DefaultTargetTemp; /* 目标温度 */

  for (int i = 0; i < LEDBUFFER_SIZE; i++)
    systemState.ledBuffer[i] = 0;

  for (int i = 0; i < TEMPBUFFER_SIZE; i++)
    systemState.tempBuffer[i] = 0;
  
  systemState.tempSum = 0;
  systemState.tempBufferIndex = 0;
  systemState.tempReadCnt = 0;
}

void handleKey(void)
{
  switch (systemState.zlg7290KeyStates.readBuffer)
  {
    /* 按键A：(Normal | Sleep有效)使targetTemp增加1，上限30 */
    case 0x19:
      if (systemState.currentState == STATE_NORMAL || systemState.currentState == STATE_SLEEP)
      {
        if (systemState.targetTemp < 30)
        {
          systemState.targetTemp++;
          updateLED();
        }
      }
      break;
    /* 按键B：(Normal | Sleep有效)使targetTemp减少1，下限16 */
    case 0x11:
      if (systemState.currentState == STATE_NORMAL || systemState.currentState == STATE_SLEEP)
      {
        if (systemState.targetTemp > 16)
        {
          systemState.targetTemp--;
          updateLED();
        }
      }
      break;
    /* 按键D：(始终有效)power按钮，进行Normal/Sleep和PowerOff之间的状态切换 */
    case 0x01:
      systemState.zlg7290KeyStates.powerBtnPressed = 1;
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
  switch (systemState.currentState)
  {
    case STATE_NORMAL:
      /* 执行关机逻辑 */
      if (systemState.zlg7290KeyStates.powerBtnPressed == 1)
      {
        systemState.zlg7290KeyStates.powerBtnPressed = 0;

        /* 设置工作状态为PowerOff，不再从LM75A读取温度 */
        systemState.currentState = STATE_POWEROFF;

        /* 熄灭数码管 */
        turnOffLED();

        /* 关闭风扇(若本关闭，则不操作) */
        if (systemState.fanStates.currentSpeedLevel != SPEED_LEVEL_0)
        {
          /* 停转 */
          DC_Motor_Pin_Low();
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;
        }

        /* 关闭Marquee */
        turnOffMarquee();
      }
      /* 执行转换到Sleep状态逻辑 */
      else if (systemState.zlg7290KeyStates.idleTicks >= ZLG7290MaxIdleTicks)
      {
        systemState.zlg7290KeyStates.idleTicks = 0;

        /* 设置工作模式为SLEEP，之后TIM3中断读取温度不再点亮LED */
        systemState.currentState = STATE_SLEEP;
        
        /* 熄灭8段7位数码管 */
        turnOffLED();

        /* 关闭Marquee */
        turnOffMarquee();
      }
      break;
    case STATE_SLEEP:
      /* 执行关机逻辑 */
      if (systemState.zlg7290KeyStates.powerBtnPressed == 1)
      {
        systemState.zlg7290KeyStates.powerBtnPressed = 0;
        systemState.zlg7290KeyStates.anyBtnPressed = 0;

        /* 设置工作状态为PowerOff，不再从LM75A读取温度 */
        systemState.currentState = STATE_POWEROFF;

        /* 关闭风扇(若本关闭，则不操作) */
        if (systemState.fanStates.currentSpeedLevel != SPEED_LEVEL_0)
        {
          /* 停转 */
          DC_Motor_Pin_Low();
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;
        }

        /* 关闭Marquee */
        turnOffMarquee();
      }
      /* 执行转换到Normal状态逻辑 */
      else if (systemState.zlg7290KeyStates.anyBtnPressed == 1)
      {
        systemState.zlg7290KeyStates.anyBtnPressed = 0;

        /* 设置工作模式为NORMAL，之后TIM3中断读取温度将点亮LED */
        systemState.currentState = STATE_NORMAL;

        updateLED();
      }
      break;
    case STATE_POWEROFF:
      /* 执行开机逻辑 */
      if (systemState.zlg7290KeyStates.powerBtnPressed == 1)
      {
        systemState.zlg7290KeyStates.powerBtnPressed = 0;

        /* 延时一段时间，让LM75A做好读取温度的准备 */
        HAL_Delay(100);

        /* 设置currentState为NORMAL 之后可以通过TIM3中断从LM75A读取温度 */
        systemState.currentState = STATE_NORMAL;
        
        while (1)
        {
          /* 无限循环直到actualTemp是一个有效温度 */
          if (systemState.actualTemp != 1)
            break;
        }

        updateLED();
                
        /* 蜂鸣器响 */
        beepOnce(BeepDelay * 10);
      }
      break;
    /* 不会出现这种情况 */
    default:
      break;
  }
}

void handleDCMotor(void)
{
  /* 如果实际温度大于目标温度，则需开启风扇或调整正在运转的风扇转速 */
  if (systemState.actualTemp > systemState.targetTemp)
  {
    const uint8_t diff = systemState.actualTemp - systemState.targetTemp;
    if (diff > 0 && diff < 2)
      systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_1;
    if (diff >= 2 && diff < 4)
      systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_2;
    if (diff >= 4)
      systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_3;

    /* 如果当前风扇转速和目标相同，则无需改变转速 */
    if (systemState.fanStates.currentSpeedLevel != systemState.fanStates.targetSpeedLevel)
    {
      /* 如果不同，改变风扇转速 */
      DC_Motor_Pin_High();
      switch (systemState.fanStates.targetSpeedLevel)
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
        /* 不会出现这种情况 */
        default:
          break;
      }
      /* 设置当前转速为目标转速 */
      systemState.fanStates.currentSpeedLevel = systemState.fanStates.targetSpeedLevel;
    }
  }
  /* 如果实际温度不大于目标温度，则停止正在运转的风扇；若风扇本就停转，则无变化 */
  else
  {
    if (systemState.fanStates.currentSpeedLevel != SPEED_LEVEL_0)
    {
      /* 停转 */
      DC_Motor_Pin_Low();
      I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
      systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;
    }
  }
}

void handleMarquee(void)
{
  /*
   * 四个小LED灯点亮与当前的风速有关
   * 同一时间只有一个灯点亮，它代表着当前风扇转速
   * 即在转速被修改后的currentSpeedLevel值
   * 在点亮前还需熄灭上一次点亮的灯
   */

  /* 熄灭当前Marquee */
  turnOffMarquee();

  /* 根据风速点亮对应Marquee */
  switch (systemState.fanStates.currentSpeedLevel)
  {
    /* 点亮D1 */
    case SPEED_LEVEL_0:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);
      systemState.currentMarqueeState = MARQUEE_D1;
      break;
    /* 点亮D2 */
    case SPEED_LEVEL_1:
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
      systemState.currentMarqueeState = MARQUEE_D2;
      break;
    /* 点亮D3 */
    case SPEED_LEVEL_2:
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
      systemState.currentMarqueeState = MARQUEE_D3;
      break;
    /* 点亮D4 */
    case SPEED_LEVEL_3:
      HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_RESET);
      systemState.currentMarqueeState = MARQUEE_D4;
      break;
    /* 不会出现这种情况 */
    default:
      break;
  }
}

/* 关闭当前Marquee */
void turnOffMarquee(void)
{
  switch (systemState.currentMarqueeState)
  {
    /* 熄灭D1 */
    case MARQUEE_D1:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
      break;
    /* 熄灭D2 */
    case MARQUEE_D2:
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
      break;
    /* 熄灭D3 */
    case MARQUEE_D3:
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
      break;
    /* 熄灭D4 */
    case MARQUEE_D4:
      HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_SET);
      break;
    /* 如果全部关闭，则不操作 */
    default:
      break;
  }

  systemState.currentMarqueeState = MARQUEE_OFF;
}

void initMarquee(void)
{
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_SET);
  systemState.currentMarqueeState = MARQUEE_OFF;
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
void assert_failed(uint8_t* file, uint32_t line)
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
