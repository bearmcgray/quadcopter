#ifndef __A7105_DRV_H
#define __A7105_DRV_H

enum A7105_State {
    A7105_SLEEP     = 0x80,
    A7105_IDLE      = 0x90,
    A7105_STANDBY   = 0xA0,
    A7105_PLL       = 0xB0,
    A7105_RX        = 0xC0,
    A7105_TX        = 0xD0,
    A7105_RST_WRPTR = 0xE0,
    A7105_RST_RDPTR = 0xF0,
};

enum A7105_MASK {
    A7105_MASK_FBCF = 1 << 4,
    A7105_MASK_VBCF = 1 << 3,
};

#define READ_BIT_FLAG 0x40
#define DUMMY_BYTE 0xff

void A7105_Initialize();
void A7105_WriteReg(u8 addr, u8 value);
void A7105_WriteRegEx(u8 addr, u8 value);

void A7105_WriteData(u8 *dpbuffer, u8 len, u8 channel);
void A7105_ReadData(u8 *dpbuffer, u8 len);
u8 A7105_ReadReg(u8 addr);
u8 A7105_ReadRegEx(u8 addr);
void A7105_Reset();
void A7105_WriteID(u32 id);
u32 A7105_ReadIDEx(void);
void A7105_WriteIDEx(u32 id);
u32 A7105_ReadID(void);
void A7105_Strobe(enum A7105_State);
int A7105_WaitWTF();
void A7105_SwitchChanel(u32 p);

#endif
