#include "RemoteInfrared.h"

#define REPEAT_KEY  0xEE

extern __IO uint32_t GlobalTimingDelay100us;
extern __IO uint32_t GlobalTimingDelay100usTx;


__IO uint32_t FlagGotKey = 0;

__IO Remote_Infrared_data_union RemoteInfrareddata;


/************************************************************************
//处理红外接收  
-------------------------协议--------------------------
开始拉低9ms,接着是一个4.5ms的高脉冲,通知器件开始传送数据了
接着是发送4个8位二进制码,第一二个是遥控识别码(REMOTE_ID),第一个为
正码(0),第二个为反码(255),接着两个数据是键值,第一个为正码
第二个为反码.发送完后40ms,遥控再发送一个9ms低,2ms高的脉冲,
表示按键的次数,出现一次则证明只按下了一次,如果出现多次,则可
以认为是持续按下该键.

*名称: Remote_Infrared_KEY_ISR(INT11_vect )													 
*功能: INT0中断服务程序		       									
*参数: 无					          									
*返回: 无		                           								
*************************************************************************/	
// 检测脉冲宽度最长脉宽为5ms
const uint32_t TIME_DELAY_6MS = 60;
const uint32_t TIME_DELAY_10MS = 100;
void Remote_Infrared_KEY_ISR(void)
{
	static __IO uint8_t  bBitCounter = 0; //键盘帧位计数
  static __IO uint32_t bKeyCode = 0;
	bBitCounter++;

	if(bBitCounter == 1)        // 开始拉低9ms
	{
        if(Remote_Infrared_DAT_INPUT) // 高电平无效
        {
            bBitCounter = 0;
        }
        else
        {
            GlobalTimingDelay100us = TIME_DELAY_10MS;
        }
	}
	else if(bBitCounter == 2)   // 4.5ms的高脉冲
	{
        if(Remote_Infrared_DAT_INPUT)
        {
            if((GlobalTimingDelay100us > 2) && (GlobalTimingDelay100us < 18))
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
	else if(bBitCounter == 3)   // 4.5ms的高脉冲
	{
        if(Remote_Infrared_DAT_INPUT)
        {
            bBitCounter = 0; 
        }
        else
        {
            if((GlobalTimingDelay100us > 5) && (GlobalTimingDelay100us < 20))
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
                printf("引导码");
            }
            else if((GlobalTimingDelay100us > 32) && (GlobalTimingDelay100us < 46))
            {
                bBitCounter = 0;
                RemoteInfrareddata.uiRemoteInfraredData = bKeyCode;
                //RemoteInfrareddata.uiRemoteInfraredData = REPEAT_KEY;
                bBitCounter = 0;
                FlagGotKey = 1;
            }            
            else
            {
                bBitCounter = 0; 
                //printf("%d&", GlobalTimingDelay100us);
            }          
        }
	}    
	else if(bBitCounter > 3 && bBitCounter < 68) //接收8位数据
	{  

        if(Remote_Infrared_DAT_INPUT)
        {
            if((GlobalTimingDelay100us > 50) && (GlobalTimingDelay100us < 58))
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
            }
            else
            {
                bBitCounter = 0; 
                //printf("#");
            }           
        }
        else
        {
            if((GlobalTimingDelay100us > 50) && (GlobalTimingDelay100us < 58)) // '0'
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;
		        bKeyCode <<= 1;  // MSB First 
                bKeyCode += 0x00;
            }
            else if((GlobalTimingDelay100us > 40) && (GlobalTimingDelay100us < 48)) //'1'
            {
                GlobalTimingDelay100us = TIME_DELAY_6MS;            
		        bKeyCode <<= 1;  // MSB First 
                bKeyCode += 0x01;
            }              
            else
            {
			    //test = GlobalTimingDelay100us;
                bBitCounter = 0; 
                //printf("*%d", test);
            }  
        }

       if(bBitCounter == 67)
        {
            RemoteInfrareddata.uiRemoteInfraredData = bKeyCode;
            bBitCounter = 0;
            FlagGotKey = 1;
            //printf("KeyCode = 0x%X", bKeyCode);
        }
	}
	else
	{
		bBitCounter = 0;
        //printf("KeyCode = 0x%X", bKeyCode);
	}
}

/************************************************************************
*名称: unsigned char Remote_Infrared_KeyDeCode(unsigned char bKeyCode)					 
*功能: PS2键盘解码程序��		       									    
*参数: bKeyCode 键盘码 							
*返回: 按键的ASIIC码		                           								
************************************************************************/
uint8_t Remote_Infrared_KeyDeCode(void)
{
//	uint8_t Key = 0xFF;

	if (FlagGotKey == 1)//通码
	{
        FlagGotKey = 0;
        if((RemoteInfrareddata.RemoteInfraredDataStruct.bID == (uint8_t)~ RemoteInfrareddata.RemoteInfraredDataStruct.bIDNot)
            && (RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode == (uint8_t)~ RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCodeNot))
        {
            printf("\n\r IR Receive KeyCode = 0x%02X, ", RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode);
			switch(RemoteInfrareddata.RemoteInfraredDataStruct.bKeyCode)
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

