#ifndef __check_H
#define __check_H

typedef enum {
  SPEED_LEVEL_0, /* 风扇停转 */
  SPEED_LEVEL_1, /* 档位1 */
  SPEED_LEVEL_2, /* 档位2 */
  SPEED_LEVEL_3, /* 档位3 */
}FanSpeed;

typedef enum {
  MARQUEE_OFF,  /* Marquee关闭 */
  MARQUEE_D1, /* D1被点亮 */
  MARQUEE_D2, /* D2被点亮 */
  MARQUEE_D3, /* D3被点亮 */
  MARQUEE_D4, /* D4被点亮 */
}MarqueeStatus;

typedef enum {
  STATE_NORMAL, /* 正常模式 */
  STATE_SLEEP, /* 睡眠模式 */
  STATE_POWEROFF, /* 关机模式 */
}SystemState;

SystemState getSystemState(void);
void setSystemState(SystemState val);
FanSpeed getTargetSpeedLevel(void);
void setTargetSpeedLevel(FanSpeed val);

#endif