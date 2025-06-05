#include "key.h"

struct keys key[3] =
    {
        {0x19, KEY_IDLE, 0},
        {0x11, KEY_IDLE, 0},
        {0x01, KEY_IDLE, 0}};

volatile uint8_t is_pressed = 0; // 是否有按键按下
uint8_t *buffer;

void keyScan(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        key[i].keyState = KEY_IDLE;
    }
    if (is_pressed) // 有按键按下
    {
        I2C_ZLG7290_Read(&hi2c1, 0x71, 0x01, buffer, 1);
        switch (*buffer)
        {
        case 0x19:
            key[0].keyState = KEY_PRESSED;
            break;
        case 0x11:
            key[1].keyState = KEY_PRESSED;
            break;
        case 0x01:
            key[2].keyState = KEY_PRESSED;
            break;
        default:
            break;
        }
    }
}