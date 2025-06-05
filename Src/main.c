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

// uint32_t unStartFlag __at (0x40003FF4); /* ������MagicNumber����λ�� */

uint8_t Buffer_DC[2] = {0x00, 0xff};

static uint32_t unStartFlag __attribute__((at(0x20003FF0), zero_init));

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
  if (unStartFlag != MagicNumber)
  {
    unStartFlag = MagicNumber; /* 设置MagicNumber，表示冷启动 */
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
    printf("cold start\n");
  }
  else
  {
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
    printf("hot start\n");
  }
  // }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (systemState.zlg7290KeyStates.canRead == 1)
    {
      systemState.zlg7290KeyStates.canRead = 0;
      I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &systemState.zlg7290KeyStates.readBuffer, 1);

      printf("0x%x\n", systemState.zlg7290KeyStates.readBuffer);
      printf("TargetTemp:%d\n", systemState.targetTemp);
      for (int i = 0; i < LEDBUFFER_SIZE; i++)
        printf("%d ", systemState.ledBuffer[i]);
      printf("\n");

      printf("state:%d\n", systemState.currentState);

      if (systemState.currentState == STATE_SLEEP)
        systemState.zlg7290KeyStates.anyBtnPressed = 1;

      if (systemState.currentState == STATE_NORMAL)
        systemState.zlg7290KeyStates.idleTicks = 0;

      if (systemState.currentState != STATE_POWEROFF)
        beepOnce(BeepDelay);
      handleKey();
      systemState.zlg7290KeyStates.readBuffer = 0;
    }

    handleStateMachine();

    refreshLED();

    if (systemState.currentState != STATE_POWEROFF)
      handleDCMotor();

    if (systemState.currentState == STATE_NORMAL)
      handleMarquee();

    /* ι�� */
    MX_IWDG_Refresh();

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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
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

/* ϵͳʱ���ж� */
void HAL_SYSTICK_Callback(void)
{
  /* ������ܵ�ǰ����״̬ΪNormal���޲���ʱ��δ������ֵ�����ۼ��޲���ʱ�� */
  if (systemState.currentState == STATE_NORMAL && systemState.zlg7290KeyStates.idleTicks < ZLG7290MaxIdleTicks)
    systemState.zlg7290KeyStates.idleTicks++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* ���յ�EXTI13ʱ������zlg7290������ȡ��־ */
  if (GPIO_Pin == GPIO_PIN_13)
    systemState.zlg7290KeyStates.canRead = 1;
}

void initSystemState(void)
{
  systemState.zlg7290KeyStates.readBuffer = 0;      /* ZLG7290��ֵ������ */
  systemState.zlg7290KeyStates.canRead = 0;         /* ZLG7290�Ƿ�ɶ���־ */
  systemState.zlg7290KeyStates.idleTicks = 0;       /* ZLG7290����δ������ʱ���ۼ��� */
  systemState.zlg7290KeyStates.powerBtnPressed = 0; /* ͨ��ʱ����״̬ΪPowerOff */
  systemState.zlg7290KeyStates.anyBtnPressed = 0;   /* ���ⰴť�����±�־ */

  systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_1; /* Ĭ�Ϸ���ת��Ϊ1�� */
  systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;

  systemState.currentMarqueeState = MARQUEE_OFF;
  systemState.currentState = STATE_POWEROFF; /* ͨ��ʱ����״̬ΪPowerOff */

  systemState.actualTemp = 1;                 /* ʵ���¶� */
  systemState.targetTemp = DefaultTargetTemp; /* Ŀ���¶� */

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
  /* ����A��(Normal | Sleep��Ч)ʹtargetTemp����1������30 */
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
  /* ����B��(Normal | Sleep��Ч)ʹtargetTemp����1������16 */
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
  /* ����D��(ʼ����Ч)power��ť������Normal/Sleep��PowerOff֮���״̬�л� */
  case 0x01:
    systemState.zlg7290KeyStates.powerBtnPressed = 1;
    break;
  /* �����������޲��� */
  default:
    break;
  }
}

void handleStateMachine(void)
{
  switch (systemState.currentState)
  {
  case STATE_NORMAL:
    /* ִ�йػ��߼� */
    if (systemState.zlg7290KeyStates.powerBtnPressed == 1)
    {
      systemState.zlg7290KeyStates.powerBtnPressed = 0;

      /* ���ù���״̬ΪPowerOff�����ٴ�LM75A��ȡ�¶� */
      systemState.currentState = STATE_POWEROFF;

      /* Ϩ������� */
      turnOffLED();

      /* �رշ���(�����رգ��򲻲���) */
      if (systemState.fanStates.currentSpeedLevel != SPEED_LEVEL_0)
      {
        /* ͣת */
        DC_Motor_Pin_Low();
        I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
        systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;
      }

      /* �ر�Marquee */
      turnOffMarquee();
    }
    /* ִ��ת����Sleep״̬�߼� */
    else if (systemState.zlg7290KeyStates.idleTicks >= ZLG7290MaxIdleTicks)
    {
      systemState.zlg7290KeyStates.idleTicks = 0;

      /* ���ù���ģʽΪSLEEP��֮��TIM3�ж϶�ȡ�¶Ȳ��ٵ���LED */
      systemState.currentState = STATE_SLEEP;

      /* Ϩ��8��7λ����� */
      turnOffLED();

      /* �ر�Marquee */
      turnOffMarquee();
    }
    break;
  case STATE_SLEEP:
    /* ִ�йػ��߼� */
    if (systemState.zlg7290KeyStates.powerBtnPressed == 1)
    {
      systemState.zlg7290KeyStates.powerBtnPressed = 0;
      systemState.zlg7290KeyStates.anyBtnPressed = 0;

      /* ���ù���״̬ΪPowerOff�����ٴ�LM75A��ȡ�¶� */
      systemState.currentState = STATE_POWEROFF;

      /* �رշ���(�����رգ��򲻲���) */
      if (systemState.fanStates.currentSpeedLevel != SPEED_LEVEL_0)
      {
        /* ͣת */
        DC_Motor_Pin_Low();
        I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
        systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;
      }

      /* �ر�Marquee */
      turnOffMarquee();
    }
    /* ִ��ת����Normal״̬�߼� */
    else if (systemState.zlg7290KeyStates.anyBtnPressed == 1)
    {
      systemState.zlg7290KeyStates.anyBtnPressed = 0;

      /* ���ù���ģʽΪNORMAL��֮��TIM3�ж϶�ȡ�¶Ƚ�����LED */
      systemState.currentState = STATE_NORMAL;

      updateLED();
    }
    break;
  case STATE_POWEROFF:
    /* ִ�п����߼� */
    if (systemState.zlg7290KeyStates.powerBtnPressed == 1)
    {
      systemState.zlg7290KeyStates.powerBtnPressed = 0;

      /* ��ʱһ��ʱ�䣬��LM75A���ö�ȡ�¶ȵ�׼�� */
      HAL_Delay(100);

      /* ����currentStateΪNORMAL ֮�����ͨ��TIM3�жϴ�LM75A��ȡ�¶� */
      systemState.currentState = STATE_NORMAL;

      // systemState.actualTemp = getActualTemp();

      printf("actualTemp=%d\n", systemState.actualTemp);
      while (1)
      {
        /* ����ѭ��ֱ��actualTemp��һ����Ч�¶� */
        if (systemState.actualTemp != 1)
          break;
      }

      updateLED();

      /* �������� */
      beepOnce(BeepDelay * 10);
    }
    break;
  /* �������������� */
  default:
    break;
  }
}

void handleDCMotor(void)
{
  /* ���ʵ���¶ȴ���Ŀ���¶ȣ����迪�����Ȼ����������ת�ķ���ת�� */
  if (systemState.actualTemp > systemState.targetTemp)
  {
    const uint8_t diff = systemState.actualTemp - systemState.targetTemp;
    if (diff > 0 && diff < 2)
      systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_1;
    if (diff >= 2 && diff < 4)
      systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_2;
    if (diff >= 4)
      systemState.fanStates.targetSpeedLevel = SPEED_LEVEL_3;

    /* �����ǰ����ת�ٺ�Ŀ����ͬ��������ı�ת�� */
    if (systemState.fanStates.currentSpeedLevel != systemState.fanStates.targetSpeedLevel)
    {
      /* �����ͬ���ı����ת�� */
      DC_Motor_Pin_High();
      switch (systemState.fanStates.targetSpeedLevel)
      {
      /* ����ת��1 */
      case SPEED_LEVEL_1:
        I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x0A, &Buffer_DC[1], 1);
        break;
      /* ����ת��2 */
      case SPEED_LEVEL_2:
        I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x05, &Buffer_DC[1], 1);
        break;
      /* ����ת��3 */
      case SPEED_LEVEL_3:
        I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x03, &Buffer_DC[1], 1);
        break;
      /* �������������� */
      default:
        break;
      }
      /* ���õ�ǰת��ΪĿ��ת�� */
      systemState.fanStates.currentSpeedLevel = systemState.fanStates.targetSpeedLevel;
    }
  }
  /* ���ʵ���¶Ȳ�����Ŀ���¶ȣ���ֹͣ������ת�ķ��ȣ������ȱ���ͣת�����ޱ仯 */
  else
  {
    if (systemState.fanStates.currentSpeedLevel != SPEED_LEVEL_0)
    {
      /* ͣת */
      DC_Motor_Pin_Low();
      I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
      systemState.fanStates.currentSpeedLevel = SPEED_LEVEL_0;
    }
  }
}

void handleMarquee(void)
{
  /*
   * �ĸ�СLED�Ƶ����뵱ǰ�ķ����й�
   * ͬһʱ��ֻ��һ���Ƶ������������ŵ�ǰ����ת��
   * ����ת�ٱ��޸ĺ��currentSpeedLevelֵ
   * �ڵ���ǰ����Ϩ����һ�ε����ĵ�
   */

  /* Ϩ��ǰMarquee */
  turnOffMarquee();

  /* ���ݷ��ٵ�����ӦMarquee */
  switch (systemState.fanStates.currentSpeedLevel)
  {
  /* ����D1 */
  case SPEED_LEVEL_0:
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    systemState.currentMarqueeState = MARQUEE_D1;
    break;
  /* ����D2 */
  case SPEED_LEVEL_1:
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    systemState.currentMarqueeState = MARQUEE_D2;
    break;
  /* ����D3 */
  case SPEED_LEVEL_2:
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    systemState.currentMarqueeState = MARQUEE_D3;
    break;
  /* ����D4 */
  case SPEED_LEVEL_3:
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);
    systemState.currentMarqueeState = MARQUEE_D4;
    break;
  /* �������������� */
  default:
    break;
  }
}

/* �رյ�ǰMarquee */
void turnOffMarquee(void)
{
  switch (systemState.currentMarqueeState)
  {
  /* Ϩ��D1 */
  case MARQUEE_D1:
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    break;
  /* Ϩ��D2 */
  case MARQUEE_D2:
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    break;
  /* Ϩ��D3 */
  case MARQUEE_D3:
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    break;
  /* Ϩ��D4 */
  case MARQUEE_D4:
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET);
    break;
  /* ���ȫ���رգ��򲻲��� */
  default:
    break;
  }

  systemState.currentMarqueeState = MARQUEE_OFF;
}

void initMarquee(void)
{
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET);
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
