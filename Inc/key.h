#ifndef __key_H
#define __key_H
#include "mxconstants.h"
#include "zlg7290.h"

typedef enum {
    KEY_IDLE,          // 按键空闲（已释放且稳定）
    KEY_PRESSED, // 按键被按下，等待10ms确认
    KEY_CONFIRMED,  // 按键已确认按下（去抖完成）
    // 如果需要处理按键释放的去抖，可以增加 KEY_DEBOUNCE_STATE_RELEASED_PENDING 等
} keyDebounceState;

struct keys
{
    uint8_t keyCode;
    keyDebounceState keyState;
    uint8_t judgeTimes;
};
extern struct keys key[3];
extern volatile uint8_t is_pressed; // 是否有按键按下

void keyScan(void);

#endif