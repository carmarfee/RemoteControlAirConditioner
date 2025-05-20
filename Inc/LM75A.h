#ifndef __LM75A_H
#define __LM75A_H

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define DEBUG 0

#define TEMP_ADDR 0x00  // �¶ȼĴ���ָ���ַ
#define CONF_ADDR 0x01  // ���üĴ���ָ���ַ
#define THYST_ADDR 0x02 // �ͺ�Ĵ���ָ���ַ
#define TOS_ADDR 0x03   // ���ȹضϼĴ���ָ���ַ

/*��������ģʽ*/
#define SHUTDOWN_MODE 0x01 // �ض�ģʽ
#define NORMOR_MODE 0x00   // ��������ģʽ
/*OS����ģʽ*/
#define OSIRQ_MODE 0x02  // OS�ж�ģʽ
#define OSCOMP_MODE 0x00 // OS�Ƚ���ģʽ
/*OS����ѡ��*/
#define OS_HIGH 0x04 // OS�ߵ�ƽ��Ч
#define OS_LOW 0x00  // OS�͵�ƽ��Ч
/*OS���϶���*/
#define EQ_DEFAULT 0x00 // Ĭ��ֵ

#define EVL_OK 0 // �ɹ�����ֵ
#define EVL_ER 1 // ʧ�ܷ���ֵ

uint8_t LM75SetMode(uint8_t ConfReg, uint8_t Mode);
uint16_t LM75GetTempReg(void);
double LM75GetTempValue(uint16_t tempreg);

void LM75A_TimerReadTemperature(void);
extern volatile uint16_t LM75A_Temperature;
extern volatile uint8_t LM75A_ReadReady;

#endif
