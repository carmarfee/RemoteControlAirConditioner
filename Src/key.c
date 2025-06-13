#include "mxconstants.h"
#include "zlg7290.h"
#include "i2c.h"

void keyScan(void)
{
  /* 每一次扫描前，重置按键状态 */
  for (uint8_t i = 0; i < KeyTotalCount; i++)
    systemState.zlg7290KeyStates.zlg7290KeyDebounce[i].keyState = KEY_IDLE;

  /* 按键按下时 */
  I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, &systemState.zlg7290KeyStates.debounceBuffer, 1);

  // printf("buffer:%d\n", systemState.zlg7290KeyStates.debounceBuffer);

  switch (systemState.zlg7290KeyStates.debounceBuffer)
  {
  case 0x19:
    systemState.zlg7290KeyStates.zlg7290KeyDebounce[0].keyState = KEY_PRESSED;
    systemState.zlg7290KeyStates.zlg7290KeyDebounce[0].keyInvoked = 1;
    break;
  case 0x11:
    systemState.zlg7290KeyStates.zlg7290KeyDebounce[1].keyState = KEY_PRESSED;
    systemState.zlg7290KeyStates.zlg7290KeyDebounce[1].keyInvoked = 1;
    break;
  case 0x01:
    systemState.zlg7290KeyStates.zlg7290KeyDebounce[2].keyState = KEY_PRESSED;
    systemState.zlg7290KeyStates.zlg7290KeyDebounce[2].keyInvoked = 1;
    break;
  default:
    /* 未定义的按键，不处理 */
    break;
  }

  /* 清理读缓冲区 */
  systemState.zlg7290KeyStates.debounceBuffer = 0;
}

void keyDebounce(uint8_t keyIndex)
{
  if (systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].keyInvoked == 1)
  {
    switch (systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].keyState)
    {
    case KEY_IDLE:
      /* 只判断了一次，第二次检测未pressed,判定为抖动;判断两次后进入idle,说明按键释放,为短按 */
      systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].judgeTimes = 0;
      systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].keyInvoked = 0;
      // printf("%d IDLE:judgeTimes:%d\n", keyIndex, systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].judgeTimes);
      break;
    case KEY_PRESSED:
      if (systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].judgeTimes == 0)
        systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].judgeTimes = 1; // 判断一次,进入待确认状态
      else if (systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].judgeTimes == 1)
      {
        systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].judgeTimes = 2; // 判断两次,进入确认状态
        systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].keyState = KEY_CONFIRMED;
        systemState.zlg7290KeyStates.canRead = 1;
        systemState.zlg7290KeyStates.readBuffer = systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].keyCode;
        // printf("accepted keyNumber:%d\n", systemState.zlg7290KeyStates.readBuffer);
      }
      // printf("%d Pressed:judgeTimes:%d\n", keyIndex, systemState.zlg7290KeyStates.zlg7290KeyDebounce[keyIndex].judgeTimes);
      break;
    default:
      Error_Handler();
      break;
    }
  }
}
