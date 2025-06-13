#ifndef __security_H
#define __security_H
#include "mxconstants.h"

/************************************** 数据备份 **************************************/

typedef struct
{
    SystemStateBlock state_data;
    uint16_t checksum;
} NVMSystemStateRecord;


extern NVMSystemStateRecord backupState0;
extern NVMSystemStateRecord backupState1;
// extern NVMSystemStateRecord systemStateRecord;

void initSystemState(); // 多备份加载失败使用init

void saveSystemState();
void loadSystemState();

uint16_t crc16(uint8_t *data, uint16_t length);

// extern uint8_t saveDirtFlag;
// extern uint8_t loadDirtFlag;

#endif
