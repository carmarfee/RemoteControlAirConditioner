#include "mxconstants.h"
static uint32_t calculateStateCrc(const SystemStateBlock *state) ///计算校验和
{
    // 排除checksum字段自身（不参与计算）
    const size_t data_size = (uint8_t*)&state->checksum - (uint8_t*)state;
    const uint8_t *data = (const uint8_t*)state;
    
    // CRC32多项式 (0x04C11DB7)
    const uint32_t polynomial = 0xEDB88320; // 反转后的标准多项式
    uint32_t crc = 0xFFFFFFFF; // 初始值
    
    // 逐字节处理数据
    for (size_t i = 0; i < data_size; i++) {
        crc ^= data[i];
        
        // 处理每个字节的8位
        for (int j = 0; j < 8; j++) {
            uint32_t mask = -(crc & 1);
            crc = (crc >> 1) ^ (polynomial & mask);
        }
    }
    
    return ~crc; // 最终取反
}
uint8_t judgeVerify(const SystemStateBlock *state)//判断某个状态(包括系统状态和备份状态）是否可靠，用于需要读取状态的时候。
{
    // 计算临时校验和（需要临时修改结构体）
    SystemStateBlock temp = *state;
    temp.checksum = 0; // 清零当前校验和
    
    uint32_t tempCheckSum = calculateStateCrc(&temp);
    
    // 比较并返回结果
    return (state->checksum == tempCheckSum) ? 1 : 0;
}

SystemStateBlock systemStateError(const SystemStateBlock *state1, 
                                 const SystemStateBlock *state2, 
                                 const SystemStateBlock *state3)//当系统状态不可靠时，从备份状态中寻找可靠状态,若无可靠状态，说明系统遭受攻击，进入错误处理。
{
    // 检查状态1是否有效
    if (state1 != NULL && judgeVerify(state1)) {
        return *state1;
    }
    
    // 检查状态2是否有效
    if (state2 != NULL && judgeVerify(state2)) {
        return *state2;
    }
    
    // 检查状态3是否有效
    if (state3 != NULL && judgeVerify(state3)) {
        return *state3;
    }
    
    // 所有状态都无效，进入错误处理
    Error_Handler();
}

uint8_t findright(const SystemStateBlock *state1,const SystemStateBlock *state2,const SystemStateBlock *state3)///主数据和备份数据仲裁，投票机制尝试找到可靠数据,找不到则进入错误处理。
{
  // 1. 检查每个状态的有效性
    const uint8_t valid1 = (state1 != NULL) ? judgeVerify(state1) : 0;
    const uint8_t valid2 = (state2 != NULL) ? judgeVerify(state2) : 0;
    const uint8_t valid3 = (state3 != NULL) ? judgeVerify(state3) : 0;
    
    // 2. 所有状态都有效的情况 - 进行关键字段投票
    if (valid1 && valid2 && valid3) {
        // 定义关键字段比较函数
        #define KEY_FIELDS_EQUAL(a, b) ( \
            memcmp(&(a)->zlg7290KeyStates, &(b)->zlg7290KeyStates, sizeof(struct ZLG7290KeyStates)) == 0 && \
            memcmp(&(a)->fanStates, &(b)->fanStates, sizeof(struct FanStates)) == 0 && \
            (a)->currentMode == (b)->currentMode && \
            (a)->targetTemp == (b)->targetTemp \
        )
        
        // 检查三个状态的关键字段是否一致
        if (KEY_FIELDS_EQUAL(state1, state2) && KEY_FIELDS_EQUAL(state2, state3)) {
            return 0; // 所有状态一致，返回主状态
        }
        
        // 两两比较
        if (KEY_FIELDS_EQUAL(state1, state2)) return 0; // 主状态和备份1一致
        if (KEY_FIELDS_EQUAL(state1, state3)) return 0; // 主状态和备份2一致
        if (KEY_FIELDS_EQUAL(state2, state3)) return 1; // 备份1和备份2一致
        
        // 没有一致的状态，返回优先级最高的有效状态
        return 0; // 主状态优先
    }
    
    // 3. 只有两个有效状态的情况
    if (valid1 && valid2) {
        // 比较关键字段
        if (memcmp(&state1->zlg7290KeyStates, &state2->zlg7290KeyStates, sizeof(struct ZLG7290KeyStates)) == 0 &&
            memcmp(&state1->fanStates, &state2->fanStates, sizeof(struct FanStates)) == 0 &&
            state1->currentMode == state2->currentMode &&
            state1->targetTemp == state2->targetTemp) {
            return 0; // 关键字段一致，返回主状态
        }
        // 不一致时返回主状态
        return 0;
    }
    
    if (valid1 && valid3) {
        if (memcmp(&state1->zlg7290KeyStates, &state3->zlg7290KeyStates, sizeof(struct ZLG7290KeyStates)) == 0 &&
            memcmp(&state1->fanStates, &state3->fanStates, sizeof(struct FanStates)) == 0 &&
            state1->currentMode == state3->currentMode &&
            state1->targetTemp == state3->targetTemp) {
            return 0; // 关键字段一致，返回主状态
        }
        return 0;
    }
    
    if (valid2 && valid3) {
        if (memcmp(&state2->zlg7290KeyStates, &state3->zlg7290KeyStates, sizeof(struct ZLG7290KeyStates)) == 0 &&
            memcmp(&state2->fanStates, &state3->fanStates, sizeof(struct FanStates)) == 0 &&
            state2->currentMode == state3->currentMode &&
            state2->targetTemp == state3->targetTemp) {
            return 1; // 关键字段一致，返回备份1
        }
        // 不一致时返回备份1
        return 1;
    }
    
    // 4. 只有一个有效状态的情况 - 返回该状态
    if (valid1) return 0;
    if (valid2) return 1;
    if (valid3) return 2;
    
    // 5. 没有有效状态 - 进入错误处理
    // 尝试使用systemStateError恢复状态
    SystemStateBlock recovered = systemStateError(state1, state2, state3);
    
    // 如果systemStateError返回了状态（可能通过Error_Handler恢复）
    // 确定哪个状态与恢复的状态匹配
    if (state1 != NULL && memcmp(state1, &recovered, sizeof(SystemStateBlock)) == 0) return 0;
    if (state2 != NULL && memcmp(state2, &recovered, sizeof(SystemStateBlock)) == 0) return 1;
    if (state3 != NULL && memcmp(state3, &recovered, sizeof(SystemStateBlock)) == 0) return 2;
    
    // 如果无法匹配任何输入状态，返回0xFF表示失败
    return 0xFF;
}
uint8_t validateAndRecoverState(const SystemStateBlock *state1,const SystemStateBlock *state2,const SystemStateBlock *state3)///验证并恢复工作状态
{
    // 检查主状态是否有效
    if (judgeVerify(state1)) {
        return 1; // 主状态有效
    }
    
    // 主状态无效，尝试恢复
    SystemStateBlock recovered = systemStateError(
        state1, 
        state2, 
        state3
    );
    
    // 如果恢复了有效状态，更新系统状态
    if (judgeVerify(&recovered)) {
         state1 = recovered;
        return 2; // 从备份恢复成功
    }
    
    // 完全恢复失败，初始化所有状态
    initState();
    return 0;
}
void refreshState()
{
    backupState1=systemState;
    backupState2=systemState;
}
