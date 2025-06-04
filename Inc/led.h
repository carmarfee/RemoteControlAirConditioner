#ifndef __LED_H

#define __LED_H

void initLED(void); /* 初始化LED */
void updateLED(void); /* 更新ledBuffer */
void refreshLED(void); /* 用ledBuffer刷新LED */
void turnOffLED(void); /* 关闭LED */

#endif