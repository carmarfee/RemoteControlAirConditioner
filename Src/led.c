#include "zlg7290.h"
#include "i2c.h"

#if DEBUG
    #include "stdio.h"
#endif

/* 数码管起始写地址 */
#define ZLG_WRITE_ADDRESS_BEGIN 0x10

/* LED_Buffer大小 */
#define BUFFER_SIZE_LED (sizeof(LED_Buffer) / sizeof(*LED_Buffer))

/* 索引对应值为数码管编码 */
const uint8_t seg7code[10]={ 0xFC,0x0C,0xDA,0xF2,0x66,0xB6,0xBE,0xE0,0xFE,0xE6};

/* 在内存中定义一块缓冲区用于保存8个数码管状态，每次更新数码管均使用这片缓冲区中的数据 */
uint8_t LED_Buffer[8]={0};

/* 声明库函数 */
static uint8_t setLEDBuffer(uint8_t index, uint8_t value);
static void updateLED();

/*******************************************************************************
* Function Name  : initLED
* Description    : 接口函数 - 初始化LED
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
* Description    : 接口函数 - 用指定的室温更新LED
* Input          : target - 取值整数16-30
* Output         : None
* Return         : OK - 0
*                  Error - 1
* Attention      : None
*******************************************************************************/
uint8_t updateLED_T(uint8_t target) {
    if (target < 16 || target > 30) {
        #if DEBUG
                printf("updateLED_T() - 输入的参数有误!\n");
        #endif
        return 1;
    }

    const uint8_t ten_digit = target / 10;
    const uint8_t single_digit = target % 10;

    uint8_t is_revised = 0;
    if (LED_Buffer[2] != ten_digit) {
        if (setLEDBuffer(2, ten_digit)) {
            #if DEBUG
                printf("updateLED_T() - setLEDBuffer(2, ten_digit)操作出错了!\n");
            #endif
            return 1;
        }
            
        is_revised = 1;
    }
    if (LED_Buffer[3] != single_digit) {
        if (setLEDBuffer(3, single_digit)) {
            #if DEBUG
                printf("updateLED_T() - setLEDBuffer(3, single_digit)操作出错了!\n");
            #endif
            return 1;
        }
            
        is_revised = 1;
    }
    
    if (is_revised) {
        updateLED();
    }
        
    return 0;
}

/*******************************************************************************
* Function Name  : updateLED_A
* Description    : 接口函数 - 用实际的室温更新LED
* Input          : actual - 取值整数0-50
* Output         : None
* Return         : OK - 0
*                  Error - 1
* Attention      : None
*******************************************************************************/
uint8_t updateLED_A(uint8_t actual) {
    if (actual > 50) {
        #if DEBUG
                printf("updateLED_A() - 输入的参数有误!\n");
        #endif
        return 1;
    }

    const uint8_t ten_digit = actual / 10;
    const uint8_t single_digit = actual % 10;
    
    uint8_t is_revised = 0;
    if (LED_Buffer[6] != ten_digit) {
        if (setLEDBuffer(6, ten_digit)) {
            #if DEBUG
                printf("updateLED_A() - setLEDBuffer(6, ten_digit)操作出错了!\n");
            #endif
            return 1;
        }
            
        is_revised = 1;
    }
    if (LED_Buffer[7] != single_digit) {
        if (setLEDBuffer(7, single_digit)) {
            #if DEBUG
                printf("updateLED_A() - setLEDBuffer(7, single_digit)操作出错了!\n");
            #endif
            return 1;
        }
            
        is_revised = 1;
    }
    
    if (is_revised) {
        updateLED();
    }
        
    return 0;
}

/*******************************************************************************
* Function Name  : setLEDBuffer
* Description    : 库函数 - 设置LED_Buffer中某一位的值
* Input          : index - 取值0-7，LED_Buffer索引值
*                  value - 取值0-9，LED_Buffer值(seg7code索引值)
* Output         : None
* Return         : OK - 0
*                  Error - 1
* Attention      : None
*******************************************************************************/

static uint8_t setLEDBuffer(uint8_t index, uint8_t value) {
    if (!(index >= 0 && index <= 7) || !(value >= 0 && value <= 9)) {
        #if DEBUG
            #include "stdio.h"
            printf("setLEDBuffer() - 输入的参数有误!\n");
        #endif
        return 1;
    }
    LED_Buffer[index] = seg7code[value];
    return 0;
}

/*******************************************************************************
* Function Name  : updateLED
* Description    : 库函数 - 使用LED_Buffer中的编码更新数码管
* Input          : 
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/

static void updateLED() {
	I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS_BEGIN,LED_Buffer,BUFFER_SIZE_LED);
}