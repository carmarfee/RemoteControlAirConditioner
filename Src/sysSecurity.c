#include "sysSecurity.h"

/************************************** 数据备份 **************************************/

void saveSystemState()
{
    memcpy(&backupState0.state_data, &systemState, sizeof(SystemStateBlock));
    backupState0.checksum = crc16((uint8_t *)&backupState0.state_data, sizeof(SystemStateBlock));
    memcpy(&backupState1.state_data, &systemState, sizeof(SystemStateBlock));
    backupState1.checksum = crc16((uint8_t *)&backupState1.state_data, sizeof(SystemStateBlock));
}

uint16_t calculated_crc16;

void loadSystemState()
{
    // 先从主备份恢复
    calculated_crc16 = crc16((uint8_t *)&backupState0.state_data, sizeof(SystemStateBlock));
    if (calculated_crc16 == backupState0.checksum)
    {
        memcpy(&systemState, &backupState0.state_data, sizeof(SystemStateBlock));
        return;
    }
    // 不行则从次备份恢复
    calculated_crc16 = crc16((uint8_t *)&backupState1.state_data, sizeof(SystemStateBlock));
    if (calculated_crc16 == backupState1.checksum)
    {
        memcpy(&systemState, &backupState1.state_data, sizeof(SystemStateBlock));
        return;
    }
    // 都失败则初始化
    initSystemState();
}

uint16_t crc16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF; // 初始值
    const uint16_t polynomial = 0x1021;

    for (uint8_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8; // 将当前字节移入CRC的高8位

        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x8000)
            { // 如果最高位是1
                crc = (crc << 1) ^ polynomial;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}
