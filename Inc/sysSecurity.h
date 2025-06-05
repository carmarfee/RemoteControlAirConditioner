#ifndef __security_H
#define __security_H
#include "mxconstants.h"

/************************************** 数据备份 **************************************/

#define BACKUP1_ADDRESS 0x20004000 // 备份区域1
#define BACKUP2_ADDRESS 0x20004010 // 备份区域2

typedef struct
{
    SystemStateBlock state_data;
    uint16_t checksum;
} NVMSystemStateRecord;

extern NVMSystemStateRecord backupState0;
extern NVMSystemStateRecord backupState1;

void initSystemState(); // 多备份加载失败使用init

void saveSystemState();
void loadSystemState();

uint16_t crc16(const uint8_t *data, size_t length);

/************************************** 硬件配置刷新 **************************************/
#define REFRESH_INTERVAL 30000



#endif
