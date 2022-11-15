
#ifndef __A5133_hal_H__
#define __A5133_hal_H__

#include "stdbool.h"
#include "stdint.h"
#include "myRadio_gpio.h"

/*********************************************************************
**  function Declaration
*********************************************************************/

void Err_State(void);
void RF_Reset(void);
void RF_WriteID(uint8_t* ptr);
void RF_ReadID(uint8_t* ptr);
uint8_t RF_ReadPage(uint8_t addr, uint8_t page);
void RF_WritePage(uint8_t addr, uint8_t wbyte, uint8_t page);
void RF_WriteReg(uint8_t, uint8_t);
uint8_t RF_ReadReg(uint8_t);
void ByteSend(uint8_t src);
uint8_t ByteRead(void);
void RF_SetCH(uint8_t);
uint8_t RF_Init(void);
void RF_FIFOWrite(uint8_t *buf, uint8_t len);
void RF_FIFORead(uint8_t *buf, uint8_t len);
void RF_StrobeCmd(uint8_t);
void RxPacket(void);
uint8_t RF_Cal_CHGroup(uint8_t ch);
uint8_t RF_Cal(void);
void RF_Config(void);
void RF_FCB(void);
void RF_KeyData(void);
void RF_FIFOLength(uint16_t len);
uint16_t RF_GetFIFOLength(void);
void RF_TrimmedValue_Init(void);
int16_t RF_RSSI_Read(void);
void RF_PM_SleepMode(void);
uint8_t RF_LVR_Check(void);
void RF_WOR_En(void);
#endif //
