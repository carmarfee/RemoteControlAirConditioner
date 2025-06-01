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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define ZLG7290MaxIdleTicks 60000 /* ZLG7290?????????????????? */
#define BeepDelay 40 /* ????????? */
#define DefaultTargetTemp 24 /* ????????? */

typedef enum {
  SPEED_LEVEL_0, /* ?????? */
  SPEED_LEVEL_1, /* ????1 */
  SPEED_LEVEL_2, /* ????2 */
  SPEED_LEVEL_3, /* ????3 */
}FanSpeed;

typedef enum {
  MARQUEE_OFF,  /* Marquee??? */
  MARQUEE_D1, /* D1?????? */
  MARQUEE_D2, /* D2?????? */
  MARQUEE_D3, /* D3?????? */
  MARQUEE_D4, /* D4?????? */
}MarqueeStatus;

typedef enum {
  STATE_NORMAL, /* ?????? */
  STATE_SLEEP, /* ????? */
  STATE_POWEROFF, /* ????? */
}SystemState;

uint8_t zlg7290ReadBuffer = 0; /* ZLG7290????????? */
uint8_t zlg7290CanRead = 0; /* ZLG7290???????? */
uint16_t zlg7290IdleTicks = 0; /* ZLG7290???????????????????? */

SystemState currentState = STATE_POWEROFF; /* ???????????PowerOff */
FanSpeed targetSpeedLevel = SPEED_LEVEL_1; /* ??????????1?? */
FanSpeed currentSpeedLevel = SPEED_LEVEL_0;
MarqueeStatus currentMarqueeStatus = MARQUEE_OFF;

// 校验值变量定义，同时初始化
static uint8_t currentState_checksum = (uint8_t) currentState ^ 0xA5;
static uint16_t targetSpeedLevel_checksum = (uint16_t) targetSpeedLevel ^ 0xA5;
static uint16_t currentSpeedLevel_checksum = (uint16_t) currentSpeedLevel ^ 0xA5;

uint8_t powerBtnPressed = 0; /* power???????????? */
uint8_t anyBtnPressed = 0; /* ?????????????? */

uint8_t actualTemp = 1; /* ?????? */
uint8_t targetTemp = DefaultTargetTemp; /* ?????? */

uint32_t unStartFlag __at (0x40003FF4); /* ??????MagicNumber???????? */

const uint8_t Buffer_DC[2] = {0x00, 0xff};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch, FILE *f);
void handleKey(void);
void handleStateMachine(void);
void handleDCMotor(void);
void handleMarquee(void);
void turnOffMarquee(void);
void initMarquee(void);
// 校验函数声明
SystemState getSystemState(void);
void setSystemState(SystemState val);
FanSpeed getTargetSpeedLevel(void);
void setTargetSpeedLevel(FanSpeed val);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  if(unStartFlag!=0xAA55AA55)//??????????
  {
    unStartFlag=0xAA55AA55;
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
  LM75SetMode(CONF_ADDR, SHUTDOWN_MODE);
  initLED();
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
    /* ??????????? */
    if (zlg7290CanRead == 1)
    {
      /* ???????????????? */
      zlg7290CanRead = 0;
      I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &zlg7290ReadBuffer, 1);

      /* ???????????????????????????????????? */
      if (currentState == STATE_SLEEP)
        anyBtnPressed = 1;
      if (currentState == STATE_NORMAL)
        zlg7290IdleTicks = 0;
      if (currentState != STATE_POWEROFF)
        beepOnce(BeepDelay);

      /* ???????????????????????????? */
      handleKey();

      /* ???????????????????????????? */
      zlg7290ReadBuffer = 0;
    }

    /* ???????????? */
    handleStateMachine();

    
    if (currentState != STATE_POWEROFF)
    {
      /* ????????????actualTemp???? */
      HAL_Delay(100);

      /* ???????????????? */
      handleDCMotor();
    }

    /* ????Marquee */
    if (currentState == STATE_NORMAL)
      handleMarquee();

    /* ???? */
    MX_IWDG_Refresh();
      
    /* ???????????????????? */
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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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

/* ????????? */
void HAL_SYSTICK_Callback(void)
{
  /* ????????????????Normal?????????????????????????????????? */
  if (currentState == STATE_NORMAL && zlg7290IdleTicks < ZLG7290MaxIdleTicks)
    zlg7290IdleTicks++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* ?????EXTI13???????zlg7290?????????? */
  if (GPIO_Pin == GPIO_PIN_13)
    zlg7290CanRead = 1;
}

void handleKey(void)
{
  switch (zlg7290ReadBuffer)
  {
    /* ????A??(Normal | Sleep????)?targetTemp????1??????30 */
    case 0x19:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        if (targetTemp < 30)
          updateLED_T(++targetTemp);
      }
      break;
    /* ????B??(Normal | Sleep????)?targetTemp????1??????16 */
    case 0x11:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        if (targetTemp > 16)
          updateLED_T(--targetTemp);
      }
      break;
    /* ????1??(Normal | Sleep????)????fanSpeedLevel?SPEED_LEVEL_1 */
    case 0x1C:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        targetSpeedLevel = SPEED_LEVEL_1;
      }
      break;
    /* ????2??(Normal | Sleep????)????fanSpeedLevel?SPEED_LEVEL_2 */
    case 0x1B:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        targetSpeedLevel = SPEED_LEVEL_2;
      }
      break;
    /* ????3??(Normal | Sleep????)????fanSpeedLevel?SPEED_LEVEL_3 */
    case 0x1A:
      if (currentState == STATE_NORMAL || currentState == STATE_SLEEP)
      {
        targetSpeedLevel = SPEED_LEVEL_3;
      }
      break;
    /* ????D??(???????)power?????????Normal/Sleep??PowerOff?????????? */
    case 0x01:
      powerBtnPressed = 1;
      break;
    /* ??????????????? */
    default:
      break;
  }
}

void handleStateMachine(void)
{
  /* 
   * ??Normal??Sleep????????????/?????Normal/Sleep?????????
   * ?????????: ????/???????? > Normal/Sleep?????
   * ???????????/???????????Normal/Sleep
   * ??PowerOff??????????????/?????????????????????????????????????????
   */
  switch (currentState)
  {
    case STATE_NORMAL:
      /* ????????? */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;

        /* ???????????PowerOff???????LM75A?????? */
        currentState = STATE_POWEROFF;

        /* ????LM75A?????? */
        LM75SetMode(CONF_ADDR, SHUTDOWN_MODE);

        /* ???????? */
        updateLED(nullBuffer);

        /* ??????(?????????????) */
        if (currentSpeedLevel != SPEED_LEVEL_0)
        {
          /* ?? */
          DC_Motor_Pin_Low();
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          currentSpeedLevel = SPEED_LEVEL_0;
        }

        /* ???Marquee */
        turnOffMarquee();
      }
      /* ????????Sleep????? */
      else if (zlg7290IdleTicks >= ZLG7290MaxIdleTicks)
      {
        zlg7290IdleTicks = 0;

        /* ???????????SLEEP?????TIM3???????????????LED */
        currentState = STATE_SLEEP;
        
        /* ???8??7??????? */
        updateLED(nullBuffer);

        /* ???Marquee */
        turnOffMarquee();
      }
      break;
    case STATE_SLEEP:
      /* ????????? */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;
        anyBtnPressed = 0;

        /* ???????????PowerOff???????LM75A?????? */
        currentState = STATE_POWEROFF;

        /* ????LM75A?????? */
        LM75SetMode(CONF_ADDR, SHUTDOWN_MODE);

        /* ??????(?????????????) */
        if (currentSpeedLevel != SPEED_LEVEL_0)
        {
          /* ?? */
          DC_Motor_Pin_Low();
          I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
          currentSpeedLevel = SPEED_LEVEL_0;
        }

        /* ???Marquee */
        turnOffMarquee();
      }
      /* ????????Normal????? */
      else if (anyBtnPressed == 1)
      {
        anyBtnPressed = 0;

        /* ???????????NORMAL?????TIM3??????????????LED */
        currentState = STATE_NORMAL;

        /* ???????actualTemp????LED */
        updateLED_A(actualTemp);
      }
      break;
    case STATE_POWEROFF:
      /* ?????????? */
      if (powerBtnPressed == 1)
      {
        powerBtnPressed = 0;

        /* ????LM75A????????????????? */
        LM75SetMode(CONF_ADDR, NORMOR_MODE);

        /* ???????????LM75A????????????? */
        HAL_Delay(100);

        /* ????currentState?NORMAL ?????????TIM3?????LM75A???????????LED */
        currentState = STATE_NORMAL;

        while (actualTemp == 1)
        {
          /* ??????????actualTemp???????????? */
        }

        /* ??actualTemp?????????????????LED????? */
                
        /* ???????? */
        beepOnce(BeepDelay * 3);
      }
      break;
    /* ?????????????? */
    default:
      break;
  }
}

void handleDCMotor(void)
{
  /* ????????????????????????????????????????????????? */
  if (actualTemp > targetTemp)
  {
    /* ????????????????????????????????? */
    if (currentSpeedLevel != targetSpeedLevel)
    {
      /* ????????????????? */
      DC_Motor_Pin_High();
      switch (targetSpeedLevel)
      {
        /* ???????1 */
        case SPEED_LEVEL_1:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x0A,&Buffer_DC[1],1);
          break;
        /* ???????2 */
        case SPEED_LEVEL_2:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x05,&Buffer_DC[1],1);
          break;
        /* ???????3 */
        case SPEED_LEVEL_3:
          I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x03,&Buffer_DC[1],1);
          break;
        /* ?????????????? */
        default:
          break;
      }
      /* ???????????????? */
      currentSpeedLevel = targetSpeedLevel;
    }
  }
  /* ????????????????????????????????????????????????????????? */
  else
  {
    if (currentSpeedLevel != SPEED_LEVEL_0)
    {
      /* ?? */
      DC_Motor_Pin_Low();
      I2C_DC_Motor_Write(&hi2c1, DC_Motor_Addr, 0x00, &Buffer_DC[0], 1);
      currentSpeedLevel = SPEED_LEVEL_0;
    }
  }
}

void handleMarquee(void)
{
  /*
   * ?????LED?????????????????
   * ???????????????????????????????????
   * ?????????????currentSpeedLevel?
   * ?????????????????????????
   */

  /* ????Marquee */
  turnOffMarquee();

  /* ?????????????Marquee */
  switch (currentSpeedLevel)
  {
    /* ????D1 */
    case SPEED_LEVEL_0:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);
      currentMarqueeStatus = MARQUEE_D1;
      break;
    /* ????D2 */
    case SPEED_LEVEL_1:
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
      currentMarqueeStatus = MARQUEE_D2;
      break;
    /* ????D3 */
    case SPEED_LEVEL_2:
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
      currentMarqueeStatus = MARQUEE_D3;
      break;
    /* ????D4 */
    case SPEED_LEVEL_3:
      HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_RESET);
      currentMarqueeStatus = MARQUEE_D4;
      break;
    /* ?????????????? */
    default:
      break;
  }
}

/* ?????Marquee */
void turnOffMarquee(void)
{
  switch (currentMarqueeStatus)
  {
    /* ???D1 */
    case MARQUEE_D1:
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
      break;
    /* ???D2 */
    case MARQUEE_D2:
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
      break;
    /* ???D3 */
    case MARQUEE_D3:
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
      break;
    /* ???D4 */
    case MARQUEE_D4:
      HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_SET);
      break;
    /* ??????????????? */
    default:
      break;
  }

  currentMarqueeStatus = MARQUEE_OFF;
}

void initMarquee(void)
{
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_15,GPIO_PIN_SET);
  currentMarqueeStatus = MARQUEE_OFF;
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
