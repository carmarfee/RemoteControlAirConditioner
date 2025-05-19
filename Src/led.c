/* 请使用此文件代替zlg7290.c */
#include "led.h"

#define I2C_Open_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2C_Open_LONG_TIMEOUT         ((uint32_t)0xffff)

__IO uint32_t  I2CTimeout = I2C_Open_LONG_TIMEOUT;

/* 以上为原zlg7290.c文件中定义 */

#define ZLG_WRITE_ADDRESS_BEGIN 0x10
#define BUFFER_SIZE_LED (countof(LED_Buffer))
#define countof(a) (sizeof(a) / sizeof(*(a)))

/* 索引对应值为数码管编码 */
unsigned char seg7code[10]={ 0xFC,0x0C,0xDA,0xF2,0x66,0xB6,0xBE,0xE0,0xFE,0xE6};

/* 在内存中定义一块缓冲区用于保存8个数码管状态，每次更新数码管均使用这片缓冲区中的数据 */
uint8_t LED_Buffer[8]={0};

/* 声明库函数 */
void I2C_ZLG7290_WriteOneByte(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value);
void I2C_ZLG7290_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num);
uint8_t setLEDBuffer(uint8_t index, uint8_t value);
uint8_t updateLED();

/*******************************************************************************
* Function Name  : initLED
* Description    : 初始化LED
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void initLED() {
    for (int i = 0; i <= 7; i++)
        setLEDBuffer(i, 0);
    
    updateLED();
}

/*******************************************************************************
* Function Name  : updateLED_T
* Description    : 用指定的室温更新LED
* Input          : target - 取值整数16-30
* Output         : None
* Return         : OK - 0; Error - 1
* Attention      : None
*******************************************************************************/
uint8_t updateLED_T(uint8_t target) {
    if (target < 16 || target > 30) {
        return 1;
    }

    setLEDBuffer(2, target / 10);
    setLEDBuffer(3, target % 10);

    updateLED();
    return 0;
}

/*******************************************************************************
* Function Name  : updateLED_A
* Description    : 用实际的室温更新LED
* Input          : actual - 取值整数16-30
* Output         : None
* Return         : OK - 0; Error - 1
* Attention      : None
*******************************************************************************/
uint8_t updateLED_A(uint8_t actual) {
    if (actual < 16 || actual > 30) {
        return 1;
    }

    setLEDBuffer(6, actual / 10);
    setLEDBuffer(7, actual % 10);

    updateLED();
    return 0;
}

/*******************************************************************************
* Function Name  : setLEDBuffer
* Description    : 库函数，设置LED_Buffer中某一位的值
* Input          : index - 取值0-7，LED_Buffer索引值
*                  value - 取值0-9，LED_Buffer值(seg7code索引值)
* Output         : None
* Return         : OK - 0; Error - 1
* Attention      : None
*******************************************************************************/

uint8_t setLEDBuffer(uint8_t index, uint8_t value) {
    if (!(index >= 0 && index <= 7) || !(value >= 0 && value <= 9)) {
        #if DEBUG
            #include "stdio.h"
            printf("setLEDBuffer() - 输入的参数有误!");
        #endif
        return 1;
    }
    LED_Buffer[index] = seg7code[value];
    return 0;
}

/*******************************************************************************
* Function Name  : updateLED
* Description    : 库函数，使用LED_Buffer中的编码更新数码管
* Input          : 
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/

uint8_t updateLED() {
	I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS_BEGIN,LED_Buffer,BUFFER_SIZE_LED);
}

/*******************************************************************************
* Function Name  : I2C_24C64_WriteOneByte
* Description    : 库函数
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_ZLG7290_WriteOneByte(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value)
{   
	while( HAL_I2C_Mem_Write(I2Cx, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &value, 0x01, I2CTimeout) != HAL_OK ){};
}

/*******************************************************************************
* Function Name  : I2C_24C64_Write
* Description    : 库函数
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_ZLG7290_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num)
{
	while(num--)
	{
    I2C_ZLG7290_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++);
		HAL_Delay(5);
	}
}