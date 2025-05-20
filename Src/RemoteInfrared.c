#include "RemoteInfrared.h"

#define REPEAT_KEY 0xEE //重复按键特殊码

extern __IO uint32_t GlobalTimingDelay100us;//在解码过程中，用于检测信号的时间间隔。
extern __IO uint32_t GlobalTimingDelay100usTx; //在发送数据时，用于延时的时间间隔。

__IO uint32_t FlagGotKey = 0; //标志位，表示是否接收到按键码

__IO Remote_Infrared_data_union RemoteInfrareddata; //定义一个联合体，用于存储接收到的红外遥控器按键码数据


const uint32_t TIME_DELAY_6MS = 60; // 6ms
const uint32_t TIME_DELAY_10MS = 100; // 10ms

/**
 * @brief 是一个中断服务程序，用于解码红外遥控器的按键信号，并将解码后的按键码存储到 RemoteInfrareddata 中。通俗来说就是将红外信号转换为按键码。
 *
 *
 * @param 无参数输入
 */
void Remote_Infrared_KEY_ISR(void)
{
    static __IO uint8_t bBitCounter = 0; // ����֡λ����
    static __IO uint32_t bKeyCode = 0;
    bBitCounter++;

    if (bBitCounter == 1) // ��ʼ����9ms
    {
        if (Remote_Infrared_DAT_INPUT) // �ߵ�ƽ��Ч
        {
            bBitCounter = 0;
        }
        else
        {
            GlobalTimingDelay100us = TIME_DELAY_10MS;
        }
    }
    else if (bBitCounter == 2) // 4.5ms�ĸ�����
    {
        if (Remote_Infrared_DAT_INPUT)
        {
            if ((GlobalTimingDelay100us > 2) && (GlobalTimingDelay100us < 18))
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
            }
            else
            {
                bBitCounter = 0;
                printf(".");
            }
        }

        else
        {
            bBitCounter = 0;
        }
    }
    else if (bBitCounter == 3) // 4.5ms�ĸ�����
    {
        if (Remote_Infrared_DAT_INPUT)
        {
            bBitCounter = 0;
        }
        else
        {
            if ((GlobalTimingDelay100us > 5) && (GlobalTimingDelay100us < 20))
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
                printf("������");
            }
            else if ((GlobalTimingDelay100us > 32) && (GlobalTimingDelay100us < 46))
            {
                bBitCounter = 0;
                RemoteInfrareddata.uiRemoteInfraredData = bKeyCode;
                // RemoteInfrareddata.uiRemoteInfraredData = REPEAT_KEY;
                bBitCounter = 0;
                FlagGotKey = 1;
            }
            else
            {
                bBitCounter = 0;
                // printf("%d&", GlobalTimingDelay100us);
            }
        }
    }
    else if (bBitCounter > 3 && bBitCounter < 68) // ����8λ����
    {

        if (Remote_Infrared_DAT_INPUT)
        {
            if ((GlobalTimingDelay100us > 50) && (GlobalTimingDelay100us < 58))
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
            }
            else
            {
                bBitCounter = 0;
                // printf("#");
            }
        }
        else
        {
            if ((GlobalTimingDelay100us > 50) && (GlobalTimingDelay100us < 58)) // '0'
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
                bKeyCode <<= 1; // MSB First
                bKeyCode += 0x00;
            }
            else if ((GlobalTimingDelay100us > 40) && (GlobalTimingDelay100us < 48)) //'1'
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
                bKeyCode <<= 1; // MSB First
                bKeyCode += 0x01;
            }
            else
            {
                // test = GlobalTimingDelay100us;
                bBitCounter = 0;
                // printf("*%d", test);
            }
        }

        if (bBitCounter == 67)
        {
            RemoteInfrareddata.uiRemoteInfraredData = bKeyCode;
            bBitCounter = 0;
            FlagGotKey = 1;
            // printf("KeyCode = 0x%X", bKeyCode);
        }
    }
    else
    {
        bBitCounter = 0;
        // printf("KeyCode = 0x%X", bKeyCode);
    }
}

/**
 * @brief 将 RemoteInfrareddata 中存储的按键码解析成对应的 ASCII 字符或其他表示形式。通俗来说就是按键码编码为ASCII码,并输出调试信息
 *
 *
 *@param 无参数输入
 *@ return 
 */
uint8_t Remote_Infrared_KeyDeCode(void)
{
    //	uint8_t Key = 0xFF;

    if (FlagGotKey == 1) // ͨ��
    {
        FlagGotKey = 0;
        if ((RemoteInfrareddata.RemoteInfraredDataStruct.bID == (uint8_t)~RemoteInfrareddata.RemoteInfraredDataStruct.bIDNot) && (RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode == (uint8_t)~RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCodeNot))
        {
            printf("\n\r IR Receive KeyCode = 0x%02X, ", RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode);
            switch (RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode)
            {
            case 0:
                printf("ERROR  ");
                break;
            case 0x78:
                printf("删除   ");
                break;
            case 0x38:
                printf("HTML5/FLASH    ");
                break;

            case 0xB8:
                printf("0      ");
                break;
            case 0x08:
                printf("1      ");
                break;
            case 0x88:
                printf("2      ");
                break;
            case 0x48:
                printf("3      ");
                break;
            case 0xC8:
                printf("4      ");
                break;
            case 0x28:
                printf("5      ");
                break;
            case 0xA8:
                printf("6      ");
                break;
            case 0xE8:
                printf("7      ");
                break;
            case 0x18:
                printf("8      ");
                break;
            case 0x98:
                printf("9      ");
                break;
            default:
                printf("Unknown key!");
            }
        }
        else
        {
            printf("\n\r ERR 0x%08X", RemoteInfrareddata.uiRemoteInfraredData);
        }
    }
    //    return Key = RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode;
    return RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode;
}
