#include "i2c.h"
#include "zlg7290.h"
#include "mxconstants.h"
#include "sysSecurity.h"

#define ZLG_WRITE_ADDRESS_BEGIN 0x10 /* 数码管起始写地址 */

/* 数码管编码表 */
const uint8_t seg7code[10] = {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6};
void updateLED(void);

static void setLEDBuffer(uint8_t index, uint8_t value)
{
  systemState.ledBuffer[index] = seg7code[value];
}

void updateLED(void)
{
  for (int i = 0; i <= 7; i++)
    setLEDBuffer(i, 0);

  if (systemState.ledBuffer[2] != systemState.targetTemp / 10)
    setLEDBuffer(2, systemState.targetTemp / 10);
  if (systemState.ledBuffer[3] != systemState.targetTemp % 10)
    setLEDBuffer(3, systemState.targetTemp % 10);
  if (systemState.ledBuffer[6] != systemState.actualTemp / 10)
    setLEDBuffer(6, systemState.actualTemp / 10);
  if (systemState.ledBuffer[7] != systemState.actualTemp % 10)
    setLEDBuffer(7, systemState.actualTemp % 10);

  // saveDirtFlag = 1;
}

void refreshLED(void)
{
  I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS_BEGIN, systemState.ledBuffer, LEDBUFFER_SIZE);

  // loadDirtFlag = 1;
}

void turnOffLED(void)
{
  for (int i = 0; i < LEDBUFFER_SIZE; i++)
    systemState.ledBuffer[i] = 0;

  refreshLED();
}
