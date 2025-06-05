/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define ZLG7290MaxIdleTicks 0x00020000 /* ZLG7290�����޲������ʱ����ֵ */
#define BeepDelay 10 /* ��������ʱ */
#define DefaultTargetTemp 28 /* Ĭ��Ŀ���¶� */
#define MagicNumber 0xAA55AA55

#define LEDBUFFER_SIZE 8 /* ledBuffer��С */
#define TEMPBUFFER_SIZE 10 /* tempBuffer��С */

typedef enum {
  SPEED_LEVEL_0, /* ����ͣת */
  SPEED_LEVEL_1, /* ��λ1 */
  SPEED_LEVEL_2, /* ��λ2 */
  SPEED_LEVEL_3, /* ��λ3 */
}FanState;

typedef enum {
  MARQUEE_OFF,  /* Marquee�ر� */
  MARQUEE_D1, /* D1������ */
  MARQUEE_D2, /* D2������ */
  MARQUEE_D3, /* D3������ */
  MARQUEE_D4, /* D4������ */
}MarqueeState;

typedef enum {
  STATE_NORMAL, /* ����ģʽ */
  STATE_SLEEP, /* ˯��ģʽ */
  STATE_POWEROFF, /* �ػ�ģʽ */
}SystemState;

struct ZLG7290KeyStates
{
  uint8_t readBuffer; /* ZLG7290��ֵ������ */
  uint8_t canRead; /* ZLG7290�Ƿ�ɶ���־ */
  uint32_t idleTicks; /* ZLG7290����δ������ʱ���ۼ��� */
  uint8_t powerBtnPressed; /* power��ť�����±�־ */
  uint8_t anyBtnPressed; /* ���ⰴť�����±�־ */
};

struct FanStates
{
  FanState targetSpeedLevel; /* Ĭ�Ϸ���ת��Ϊ1�� */
  FanState currentSpeedLevel;
};

typedef struct
{
  uint8_t refreshTicks;

  struct ZLG7290KeyStates zlg7290KeyStates;
  struct FanStates fanStates;
  
  MarqueeState currentMarqueeState;
  SystemState currentState; /* ͨ��ʱ����״̬ΪPowerOff */

  uint8_t actualTemp; /* ʵ���¶� */
  uint8_t targetTemp; /* Ŀ���¶� */

  uint8_t ledBuffer[LEDBUFFER_SIZE];

  uint8_t tempBuffer[TEMPBUFFER_SIZE];
  uint16_t tempSum;
  uint8_t tempBufferIndex;
  uint8_t tempReadCnt;
}SystemStateBlock;

extern SystemStateBlock systemState;

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
