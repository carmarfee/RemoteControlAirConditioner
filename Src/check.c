#include "check.h"

SystemState getSystemState(void)///异或校验读取当前工作状态
 {
    uint8_t temp_checksum = (uint8_t)currentState ^ 0xA5;
    if(temp_checksum != currentState_checksum) {
        Error_Handler();
    }
    return currentState;
}

void setSystemState(SystemState val)///将工作状态的赋值改为使用这个函数，同时会更新当前状态的检验值
{
    currentState = val;
    currentState_checksum = (uint8_t)val ^ 0xA5;
}

FanSpeed getTargetSpeedLevel(void)//异或校验读取目标转速
{
    uint16_t temp = (uint16_t)targetSpeedLevel;
    if(temp ^ 0xAA55 != targetSpeedLevel_checksum) {
        Error_Handler();
    }
    return targetSpeedLevel;
}

FanSpeed getCurrentSpeedLevel(void)//异或校验读取实时转速
{
    uint16_t temp = (uint16_t)currentSpeedLevel;
    if(temp ^ 0xAA55 != currentSpeedLevel_checksum) {
        Error_Handler();
    }
    return currentSpeedLevel;
}
void setcurrentSpeedLevel(FanSpeed val)//将实时转速设置改为使用这个函数，同时会更新实时转速的校验值 
{
    currentSpeedLevel = val;
    currentSpeedLevel_checksum = (uint16_t)val ^ 0xAA55;
}

void setTargetSpeedLevel(FanSpeed val)//将目标转速设置改为使用这个函数，同时会更新目标转速的校验值 
{
    targetSpeedLevel = val;
    targetSpeedLevel_checksum = (uint16_t)val ^ 0xAA55;
}