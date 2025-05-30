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

#define ZLG7290MaxIdleTicks 60000 /* ZLG7290�����޲������ʱ����ֵ */
#define BeepDelay 40 /* ��������ʱ */

typedef enum {
  SPEED_LEVEL_0, /* ����ͣת */
  SPEED_LEVEL_1, /* ��λ1 */
  SPEED_LEVEL_2, /* ��λ2 */
  SPEED_LEVEL_3, /* ��λ3 */
}FanSpeed;

typedef enum {
  STATE_NORMAL, /* ����ģʽ */
  STATE_SLEEP, /* ˯��ģʽ */
  STATE_POWEROFF, /* �ػ�ģʽ */
}SystemState;

uint8_t zlg7290ReadBuffer = 0; /* ZLG7290��ֵ������ */
uint8_t zlg7290CanRead = 0; /* ZLG7290�Ƿ�ɶ���־ */
uint16_t zlg7290IdleTicks = 0; /* ZLG7290����δ������ʱ���ۼ��� */

SystemState currentState = STATE_POWEROFF; /* ͨ��ʱ����״̬ΪPowerOff */
FanSpeed currentSpeedLevel = SPEED_LEVEL_1; /* Ĭ�Ϸ���ת��Ϊ1�� */
FanSpeed lastSpeedLevel = SPEED_LEVEL_0;

uint8_t powerBtnPressed = 0; /* power��ť�����±�־ */
uint8_t anyBtnPressed = 0; /* ���ⰴť�����±�־ */

uint8_t actualTemp = 1; /* ʵ���¶� */
uint8_t targetTemp = 24; /* Ŀ���¶� */

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
    /* ִ�ж���ֵ�߼� */
    if (zlg7290CanRead == 1)
    {
      /* ���ö�ȡ��־������ֵ */
      zlg7290CanRead = 0;
      I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &zlg7290ReadBuffer, 1);

      /* �ɹ���ȡ��ֵ�������޲���ʱ�䣬������������Ӧ */
      if (currentState == STATE_SLEEP)
        anyBtnPressed = 1;
      if (currentState == STATE_NORMAL)
        zlg7290IdleTicks = 0;
      if (currentState != STATE_POWEROFF)
        beepOnce(BeepDelay);

      /* ���ݶ��������еļ�ֵ��ִ�в�ͬ�߼� */
      handleKey();

      /* ���ö�������������Ӱ����һ�ζ�ȡ */
      zlg7290ReadBuffer = 0;
    }

    /* ִ��״̬�л��߼� */
    handleStateMachine();

    /* ��ʱһ��ʱ�䣬ȷ��actualTemp���� */
    HAL_Delay(100);

    /* ִ������ֱ������߼� */
    if (currentState != STATE_POWEROFF)
      handleDCMotor();

    /* ��ʱһ��ʱ�䣬��ֹ�������� */
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

/* ϵͳʱ���ж� */
void HAL_SYSTICK_Callback(void)
{
  /* ������ܵ�ǰ����״̬ΪNormal���޲���ʱ��δ������ֵ�����ۼ��޲���ʱ�� */
  if (currentState == STATE_NORMAL && zlg7290IdleTicks < ZLG7290MaxIdleTicks)
    zlg7290IdleTicks++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* ���յ�EXTI13ʱ������zlg7290������ȡ��־ */
  if (GPIO_Pin == GPIO_PIN_13)
  {
    zlg7290CanRead = 1;
  }
}

void handleKey(void)
{
  switch (zlg7290ReadBuffer)
  {
    /* ����A��(Normal | Sleep��Ч)ʹtargetTemp����1������30 */
    case 0x19:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        if (targetTemp < 30)
          updateLED_T(++targetTemp);
      }
      break;
    /* ����B��(Normal | Sleep��Ч)ʹtargetTemp����1������16 */
    case 0x11:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        if (targetTemp > 16)
          updateLED_T(--targetTemp);
      }
      break;
    /* ����1��(Normal | Sleep��Ч)����fanSpeedLevelΪSPEED_LEVEL_1 */
    case 0x1C:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        currentSpeedLevel = SPEED_LEVEL_1;
      }
      break;
    /* ����2��(Normal | Sleep��Ч)����fanSpeedLevelΪSPEED_LEVEL_2 */
    case 0x1B:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        currentSpeedLevel = SPEED_LEVEL_2;
      }
      break;
    /* ����3��(Normal | Sleep��Ч)����fanSpeedLevelΪSPEED_LEVEL_3 */
    case 0x1A:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        currentSpeedLevel = SPEED_LEVEL_3;
      }
      break;
    /* ����D��(ʼ����Ч)power��ť������Normal/Sleep��PowerOff֮���״̬�л� */
    case 0x01:
      powerBtnPressed = 1;
      break;
    /* �����������޲��� */
    default:
      break;
  }
}

void handleStateMachine(void)
{
  /* 
   * ��Normal��Sleepģʽ�£����ڿ���/�ػ���Normal/Sleep����״̬ת��
   * �������ȼ�: ����/�ػ�״̬ת�� > Normal/Sleep״̬ת��
   * ���Ҵ�����/�ػ��󣬲��ٴ���Normal/Sleep
   * ��PowerOffģʽ�£������ڿ���/�ػ�״̬ת������ͨ����־λ�ж��Ƿ���Ҫת����������
   */
  switch(currentState)
  {
    case STATE_NORMAL:
      /* ִ�йػ��߼� */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;
        /* Ϩ������� */
        updateLED(nullBuffer);

        /* �رշ���(�����رգ��򲻲���) */
        if (lastSpeedLevel != SPEED_LEVEL_0)
        {
          /* ͣת */
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          lastSpeedLevel = SPEED_LEVEL_0;
        }

        currentState = STATE_POWEROFF;
      }
      /* ִ��ת����Sleep״̬�߼� */
      else if (zlg7290IdleTicks == ZLG7290MaxIdleTicks)
      {
        zlg7290IdleTicks = 0;
        /* Ϩ��8��7λ����� */
        updateLED(nullBuffer);

        currentState = STATE_SLEEP;
      }
      break;
    case STATE_SLEEP:
      /* ִ�йػ��߼� */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;
        /* �رշ���(�����رգ��򲻲���) */
        if (lastSpeedLevel != SPEED_LEVEL_0)
        {
          /* ͣת */
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          lastSpeedLevel = SPEED_LEVEL_0;
        }

        currentState = STATE_POWEROFF;
      }
      /* ִ��ת����Normal״̬�߼� */
      else if (anyBtnPressed == 1)
      {
        anyBtnPressed = 0;
        /* ����8��7λ����� */
        updateLED(LED_Buffer);

        currentState = STATE_NORMAL;
      }
      break;
    case STATE_POWEROFF:
      /* ִ�п����߼� */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;
        /* ��ȡ��ʵ���¶ȣ����丳ֵ��actualTemp */
        actualTemp = LM75A_TimerReadTemperature();
        while (actualTemp == 1) {
          actualTemp = LM75A_TimerReadTemperature();
          HAL_Delay(20);
        }
        
        /* (��ʵ���¶ȸ���LED_Buffer��)����8��7λ����� */
        updateLED_A(actualTemp);
        
        /* �������� */
        beepOnce(BeepDelay);

        currentState = STATE_NORMAL;
      }
      break;
    default:
      /* �������������� */
      break;
  }
}

void handleDCMotor(void)
{
  /* ���ʵ���¶ȴ���Ŀ���¶ȣ����迪�����Ȼ����������ת�ķ���ת�� */
  if (actualTemp > targetTemp)
  {
    /* �����һ�η���ת�ٺ������ͬ��������ı�ת�� */
    if (lastSpeedLevel != currentSpeedLevel)
    {
      /* �����ͬ���ı����ת�� */
      DC_Motor_Pin_High();
      switch(currentSpeedLevel)
      {
        /* ����ת��1 */
        case SPEED_LEVEL_1:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x0A,&Buffer_DC[1],1);
          break;
        /* ����ת��2 */
        case SPEED_LEVEL_2:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x05,&Buffer_DC[1],1);
          break;
        /* ����ת��3 */
        case SPEED_LEVEL_3:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x03,&Buffer_DC[1],1);
          break;
        default:
          /* �������������� */
          break;
      }
      /* ������һ��ת��Ϊ��ǰת�� */
      lastSpeedLevel = currentSpeedLevel;
    }
  }
  /* ���ʵ���¶Ȳ�����Ŀ���¶ȣ���ֹͣ������ת�ķ��ȣ������ȱ���ͣת�����ޱ仯 */
  else
  {
    if (lastSpeedLevel != SPEED_LEVEL_0)
    {
      /* ͣת */
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
