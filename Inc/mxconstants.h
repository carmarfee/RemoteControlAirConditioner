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
#define ZLG7290MaxIdleTicks 0x00020000 /* ZLG7290按键无操作最大时间阈值 */
#define BeepDelay 10                   /* 蜂鸣器延时 */
#define DefaultTargetTemp 28           /* 默认目标温度 */
#define MagicNumber 0xAA55AA55

#define LEDBUFFER_SIZE 8   /* ledBuffer大小 */
#define TEMPBUFFER_SIZE 10 /* tempBuffer大小 */

#define KeyTotalCount 3 /* 设置的按键数量 */

typedef enum
{
  SPEED_LEVEL_0, /* 风扇停转 */
  SPEED_LEVEL_1, /* 档位1 */
  SPEED_LEVEL_2, /* 档位2 */
  SPEED_LEVEL_3, /* 档位3 */
} FanState;

typedef enum
{
  MARQUEE_OFF, /* Marquee关闭 */
  MARQUEE_D1,  /* D1被点亮 */
  MARQUEE_D2,  /* D2被点亮 */
  MARQUEE_D3,  /* D3被点亮 */
  MARQUEE_D4,  /* D4被点亮 */
} MarqueeState;

typedef enum
{
  STATE_NORMAL,   /* 正常模式 */
  STATE_SLEEP,    /* 睡眠模式 */
  STATE_POWEROFF, /* 关机模式 */
} SystemState;

typedef enum
{
  KEY_IDLE,      /* 按键空闲（已释放且稳定） */
  KEY_PRESSED,   /* 按键被按下，等待10ms确认 */
  KEY_CONFIRMED, /* 按键已确认按下（去抖完成） */
} keyDebounceState;

struct ZLG7290KeyDebounceStates
{
  uint8_t keyInvoked;
  uint8_t keyCode;
  keyDebounceState keyState;
  uint8_t judgeTimes;
};

struct ZLG7290KeyStates
{
  uint8_t readBuffer;                                                /* ZLG7290键值缓冲区 */
  uint8_t canRead;                                                   /* ZLG7290是否可读标志 */
  uint32_t idleTicks;                                                /* ZLG7290按键未被按下时间累加器 */
  uint8_t powerBtnPressed;                                           /* power按钮被按下标志 */
  uint8_t anyBtnPressed;                                             /* 任意按钮被按下标志 */
  uint8_t invoked;                                                   /* ZLG7290中断发生标志 */
  struct ZLG7290KeyDebounceStates zlg7290KeyDebounce[KeyTotalCount]; /* 按键防抖状态 */
  uint8_t debounceBuffer;                                            /* 按键防抖读取缓冲区 */
};

struct FanStates
{
  FanState targetSpeedLevel; /* 默认风扇转速为1档 */
  FanState currentSpeedLevel;
};

typedef struct
{
  struct ZLG7290KeyStates zlg7290KeyStates;
  struct FanStates fanStates;

  MarqueeState currentMarqueeState;
  SystemState currentState; /* 通电时工作状态为PowerOff */

  uint8_t actualTemp; /* 实际温度 */
  uint8_t targetTemp; /* 目标温度 */

  uint8_t ledBuffer[LEDBUFFER_SIZE];

  uint8_t tempBuffer[TEMPBUFFER_SIZE];
  uint16_t tempSum;
  uint8_t tempBufferIndex;
  uint8_t tempReadCnt;
} SystemStateBlock;


extern SystemStateBlock systemState;
extern uint8_t checkSum;

/* USER CODE END Private defines */

/**
 * @}
 */

/**
 * @}
 */

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
