#include "A5133_hal.h"
#include "A5133reg.h"
#include "A5133Config.h"
uint8_t timer;
uint16_t TimerCnt;
uint8_t Slot_TX;
uint8_t Flag_Timeout;
uint16_t RxCnt;
uint8_t  *Uartptr;
uint8_t UartSendCnt;
uint8_t CmdBuf[25];
uint8_t  tmpbuf[80];
uint16_t Err_Loss;
uint16_t Err_Frame;
uint32_t Err_BitCnt;
uint8_t Flag_Report;
uint8_t Flag_FirstLink;
uint8_t Flag_MASTER;
uint8_t Mem_RH;
uint8_t Mem_RL;

const uint8_t BitCount_Tab[16] = {0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};
//const uint8_t //code ID_Tab[8]={0x34,0x75,0xC5,0x2A,0xC7,0x33,0x45,0xEA}; //ID code
const uint8_t ID_Tab[8]={0x55,0x55,0x55,0x55,0x34,0x75,0xC5,0x6A}; //ID2 code
//const uint8_t ID_Tab[8]={0xFF,0xFF,0xFF,0xFF,0x36,0x75,0xC5,0xBA}; //ID2 code
//const uint8_t ID_Tab[8]={0x34,0x75,0xC5,0x2A,0x34,0x75,0xC5,0x2A}; //ID2 code
const uint8_t KeyData_Tab[16]={0x00,0x00,0x00,0x00,0x00,0x0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //keyData code
const uint8_t FCB_Tab[20]={0x00,0x00,0x00,0x00,0x00,0x15,0x20,0x25,0x30,0x35,0x40,0x45,0x50,0x55,0x60,0x65,0x70,0x75,0x80,0x85}; //keyData code
const uint8_t PN9_Tab[]=
{   0xFF,0x83,0xDF,0x17,0x32,0x09,0x4E,0xD1,
    0xE7,0xCD,0x8A,0x91,0xC6,0xD5,0xC4,0xC4,
    0x40,0x21,0x18,0x4E,0x55,0x86,0xF4,0xDC,
    0x8A,0x15,0xA7,0xEC,0x92,0xDF,0x93,0x53,
    0x30,0x18,0xCA,0x34,0xBF,0xA2,0xC7,0x59,
    0x67,0x8F,0xBA,0x0D,0x6D,0xD8,0x2D,0x7D,
    0x54,0x0A,0x57,0x97,0x70,0x39,0xD2,0x7A,
    0xEA,0x24,0x33,0x85,0xED,0x9A,0x1D,0xE0,
};

/*********************************************************************
** Err_State
*********************************************************************/
void Err_State(void)
{
   //ERR display
   //Error Proc...
   //...
   while(1);
}
   
/************************************************************************
**  Reset_RF
************************************************************************/
void RF_Reset(void)
{
    RF_WriteReg(MODE_REG, 0x00); //reset RF chip
}

/************************************************************************
**  RF_WriteID
************************************************************************/
void RF_WriteID(uint8_t* ptr)
{
    uint8_t i;

    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(IDCODE_REG);
    for (i=0; i < 8; i++)
        myRadioSpi_wByte(*ptr++);
    BOARD_A5133_SCS_H();
}

/************************************************************************
**  RF_ReadID
************************************************************************/
void RF_ReadID(uint8_t* ptr)
{
   uint8_t i;
   
    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(IDCODE_REG | 0x40);
   for (i=0; i<8; i++)
      *ptr++ = myRadioSpi_rByte();
    BOARD_A5133_SCS_H();
}

/*********************************************************************
** RF_ReadPage
*********************************************************************/
uint8_t RF_ReadPage(uint8_t addr, uint8_t page)
{
    uint8_t tmp;

    RF_WriteReg(RFANALOG_REG, (A5133_RFConfigTab_Main[0x35]&0x0F) | page<<4);
    tmp = RF_ReadReg(addr);
    return tmp;
}

/*********************************************************************
** RF_WritePage
*********************************************************************/
void RF_WritePage(uint8_t addr, uint8_t wbyte, uint8_t page)
{
    RF_WriteReg(RFANALOG_REG, (A5133_RFConfigTab_Main[0x35]&0x0F) | page<<4);
    RF_WriteReg(addr, wbyte);
}

/************************************************************************
**  RF_WriteReg
************************************************************************/
void RF_WriteReg(uint8_t addr, uint8_t dataByte)
{
   BOARD_A5133_SCS_L();
   myRadioSpi_wByte(addr);//bit7 cmd=0, bit6 r/w=0
   myRadioSpi_wByte(dataByte);
   BOARD_A5133_SCS_H();
}

/************************************************************************
**  RF_ReadReg
************************************************************************/
uint8_t RF_ReadReg(uint8_t addr)
{
   uint8_t tmp;

   BOARD_A5133_SCS_L();
   myRadioSpi_wByte(addr | 0x40);//bit7 cmd=0,bit6 r/w=1
   tmp = myRadioSpi_rByte();
   BOARD_A5133_SCS_H();

   return tmp;
}

/*********************************************************************
** RF_SetCH
*********************************************************************/
void RF_SetCH(uint8_t ch)
{
    RF_WriteReg(PLL1_REG, ch); //RF freq = RFbase + (CH_Step * ch)
}

/*********************************************************************
** initRF
*********************************************************************/
uint8_t RF_Init(void)
{
   uint8_t i,id[8];

   RF_Reset(); //reset RF chip
   RF_WriteID((uint8_t *)ID_Tab); //write ID code
   RF_ReadID(id);
   for (i=0; i<8; i++)
   {
      if (id[i] ^ ID_Tab[i])
         return 1;//fail
   }
      
   RF_Config(); //config A7157 chip
   RF_TrimmedValue_Init();//load trimming value
   if(RF_Cal()) //rf calibration
      return 1;
   
   return 0;
}

/*********************************************************************
** RF_FIFOWrite
*********************************************************************/
void RF_FIFOWrite(uint8_t *buf, uint8_t len)
{
    uint8_t i;

   RF_StrobeCmd(CMD_TFR);//TX fifo pointer reset
    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(FIFO_REG);//send address 0x05, bit7 cmd=0, bit6 r/w=0
   for(i=0; i <len; i++)
      myRadioSpi_wByte(*(buf + i));
    BOARD_A5133_SCS_H();  
}
/*********************************************************************
** RF_FIFORead
*********************************************************************/
void RF_FIFORead(uint8_t *buf, uint8_t len)
{
    uint8_t i;
	uint8_t tx[80] = {FIFO_REG | 0x40};
	uint8_t rx[80];

    RF_StrobeCmd(CMD_RFR);//RX fifo pointer reset
	
    BOARD_A5133_SCS_L();
	/*
    myRadioSpi_wByte(FIFO_REG | 0x40);//address 0x05, bit7 cmd=0, bit6 r/w=1
    for(i=0; i <len; i++)
    {
        *buf ++ = myRadioSpi_rByte();
    }
	*/
	myRadioSpi_trx(tx, rx, len+1);
    BOARD_A5133_SCS_H();
	
	memcpy(buf, rx+1, len);
}

/*********************************************************************
** RF_StrobeCmd
*********************************************************************/
void RF_StrobeCmd(uint8_t cmd)
{
    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(cmd);
    BOARD_A5133_SCS_H();
}

/*********************************************************************
** RxPacket
*********************************************************************/
void RxPacket(void)
{
   uint8_t i,recv,tmp,err;

    RxCnt++;
    err=0;

    RF_StrobeCmd(CMD_RFR);//RX fifo pointer reset
   BOARD_A5133_SCS_L();
    myRadioSpi_wByte(FIFO_REG | 0x40);//address 0x05, bit7 cmd=0, bit6 r/w=1
    for(i=0; i <64; i++)
    {
      recv = myRadioSpi_rByte();
      tmpbuf[i]=recv;
      if((recv ^ PN9_Tab[i])!=0)
      {
         tmp = recv ^ PN9_Tab[i];
         Err_BitCnt += (BitCount_Tab[tmp>>4] + BitCount_Tab[tmp & 0x0F]);
         err=1;
        }
   }
   BOARD_A5133_SCS_H();

    if (err)//packet error
        Err_Frame++;
}

/*********************************************************************
** RF_Cal_CHGroup
*********************************************************************/
uint8_t RF_Cal_CHGroup(uint8_t ch)
{
   uint8_t tmp;
   uint8_t vb,vbcf,vcb,vccf;
   uint8_t deva,adev;

   RF_WriteReg(PLL1_REG, ch);
   RF_WriteReg(CALIBRATION_REG, 0x1C);
   do{
         tmp = RF_ReadReg(CALIBRATION_REG)&0x1C;
   }while (tmp);

   //for check
   tmp = RF_ReadReg(VCOCCAL_REG);
   vcb = tmp & 0x0F;
   vccf = (tmp>>4) & 0x01;

   tmp = RF_ReadReg(VCOCAL1_REG);
   vb = tmp & 0x0F;
   vbcf = (tmp >>4) & 0x01;

   tmp = RF_ReadReg(VCODEVCAL1_REG);
   deva = tmp;

   tmp = RF_ReadReg(VCODEVCAL2_REG);
   adev = tmp;

   if(vbcf)
      return 1;//error   
   
   return 0;
}

/*********************************************************************
** calibration
*********************************************************************/
uint8_t RF_Cal(void)
{
   uint8_t tmp;
   uint8_t rhc,rlc,fb,fbcf,fcd;

   RF_StrobeCmd(CMD_PLL); //calibration @PLL state
   RF_WriteReg(RFANALOG_REG, 0);
   
   //IF,RSSI,RC procedure
   RF_WriteReg(CALIBRATION_REG, 0x23);
   do{
         tmp = RF_ReadReg(CALIBRATION_REG)&0x23;
   }while(tmp); 

   //calibration VBC,VDC procedure
   if(RF_Cal_CHGroup(25)) //calibrate channel group Bank I
      return 1;
   if(RF_Cal_CHGroup(75)) //calibrate channel group Bank II
      return 1;
   if(RF_Cal_CHGroup(125)) //calibrate channel group Bank III
      return 1;
   
   RF_StrobeCmd(CMD_STBY); //return to STBY state

   //for check
   tmp = RF_ReadReg(IFCAL1_REG);
   fb = tmp & 0x0F;
   fbcf = (tmp>>4) & 0x01;
        
   tmp = RF_ReadReg(IFCAL2_REG);
    fcd = tmp & 0x1F;

   rhc = RF_ReadReg(RXGAIN2_REG);
   rlc = RF_ReadReg(RXGAIN3_REG);
    Mem_RH = rhc;
   Mem_RL = rlc;

   if(fbcf)
      return 1;//error
   
   return 0;
}

/*********************************************************************
** RF_Config
*********************************************************************/
void RF_Config(void)
{
    uint8_t i;

   //0x00 mode register, for reset
   //0x05 fifo data register
   //0x06 id code register
   //0x36 key data, 16 bytes
   //0x3D FCB register,4 bytes
   //0x3F USID register, read only

   for (i=0x01; i<=0x04; i++)
      RF_WriteReg(i, A5133_RFConfigTab_Main[i]);

    for (i=0x07; i<=0x1F; i++)
        RF_WriteReg(i, A5133_RFConfigTab_Main[i]);

    for (i=0; i<=12; i++)//0x20 code1
        RF_WritePage(0x20, A5133_RFConfigTab_Addr0x20[i], i);

    for (i=0; i<=12; i++)//0x21 code2
        RF_WritePage(0x21, A5133_RFConfigTab_Addr0x21[i], i);

    for (i=0; i<=5; i++)//0x22 code3
        RF_WritePage(0x22, A5133_RFConfigTab_Addr0x22[i], i);

    for (i=0x23; i<=0x29; i++)
    {
        RF_WriteReg(i, A5133_RFConfigTab_Main[i]);
    }

    for (i=0; i<=12; i++)//0x2A DAS
        RF_WritePage(0x2A, A5133_RFConfigTab_Addr0x2A[i], i);

    for (i=0x2B; i<=0x35; i++)
        RF_WriteReg(i, A5133_RFConfigTab_Main[i]);

    RF_WriteReg(0x37, A5133_RFConfigTab_Main[0x37]);

    for (i=0; i<=11; i++)//0x38 ROM
        RF_WritePage(0x38, A5133_RFConfigTab_Addr0x38[i], i);

    for (i=0x39; i<=0x3C; i++)
        RF_WriteReg(i, A5133_RFConfigTab_Main[i]);

    RF_WriteReg(0x3E, A5133_RFConfigTab_Main[0x3E]);
}

/*********************************************************************
** RF_FCB
*********************************************************************/
void RF_FCB(void)
{
    uint8_t i;

   BOARD_A5133_SCS_L();
    myRadioSpi_wByte(FCB_REG);//address 0x3D, bit7 cmd=0, bit6 r/w=0
    for (i=0; i < 20; i++)
        myRadioSpi_wByte(FCB_Tab[i]);
    BOARD_A5133_SCS_H();
}

/*********************************************************************
** A7157_KeyData
*********************************************************************/
void RF_KeyData(void)
{
    uint8_t i;

    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(KEYDATA_REG);//address 0x36, bit7 cmd=0, bit6 r/w=0
    for (i=0; i < 16; i++)
        myRadioSpi_wByte(KeyData_Tab[i]);
    BOARD_A5133_SCS_H();
}

/*********************************************************************
** RF_FIFOLength
*********************************************************************/
void RF_FIFOLength(uint16_t len)
{
    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(FIFO1_REG | 0x40);//bit7 cmd=0,bit6 r/w=1
    myRadioSpi_wByte(len & 0xFF);//low byte
   myRadioSpi_wByte(len>>8);//high byte
    BOARD_A5133_SCS_H();
}
/*********************************************************************
** RF_GetFIFOLength
*********************************************************************/
uint16_t RF_GetFIFOLength(void)
{
    uint16_t len = 0;
    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(FIFO1_REG);
    len = myRadioSpi_rByte();
    len |= (uint16_t)myRadioSpi_rByte()<<8;
    BOARD_A5133_SCS_H();
    return len;
}

/*********************************************************************
** RF_TrimmedValue_Init
*********************************************************************/
void RF_TrimmedValue_Init(void)
{
   uint8_t i;
   uint8_t trimValue[8];
   //uint8_t tmp_checksum;
   
   //trimValue[0]=FBG
   //trimValue[1]=CTR
   //trimValue[2]=BDC
   //trimValue[3]=STM
   //trimValue[4]=Checksum for trimvalue[0]~trimvalue[3]
   //trimValue[5]=CSXTL
   //trimValue[6]=FBG_CP
   //trimValue[7]=Checksum for customer
   
   RF_WritePage(ROMP_REG, A5133_RFConfigTab_Addr0x38[9] | 0xA0, 9);//enable EFSW=1, EFRE=1
    BOARD_A5133_SCS_L();
    myRadioSpi_wByte(USID_REG | 0x40);
    for (i=0; i < 8; i++)
        trimValue[i] = myRadioSpi_rByte();
    BOARD_A5133_SCS_H();
   RF_WritePage(ROMP_REG, A5133_RFConfigTab_Addr0x38[9], 9);//disable EFSW=1, EFRE=1
   
   if((trimValue[0] + trimValue[1]) == trimValue[4]) //case1-only FT
   {
      if((trimValue[0]!=0) && (trimValue[1]!=0))
      {
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[1] & 0xE0) | trimValue[0], 1);//FBG
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[2] & 0xC0) | trimValue[1], 2);//CTR          
      }
      else
         Err_State();
   }
   else if((trimValue[0] + trimValue[1] + trimValue[2] + trimValue[3]) == trimValue[4]) //case2-CP+FT
   {
      if((trimValue[0]!=0) && (trimValue[1]!=0) && (trimValue[2]!=0) && (trimValue[3]!=0)) 
      {
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[1] & 0xE0) | trimValue[0], 1);//FBG
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[2] & 0xC0) | trimValue[1], 2);//CTR      
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[0] & 0x03) | (trimValue[2]<<2), 0);//BDC
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[4] & 0x40) | trimValue[3], 4);//STM
      }
      else
         Err_State();
   }
   else //only CP
   {
       if((trimValue[0]==0) && (trimValue[1]!=0) && (trimValue[2]!=0) && (trimValue[3]!=0) && (trimValue[4]==0) && (trimValue[6]!=0)) 
      {
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[1] & 0xE0) | trimValue[6], 1);//FBG
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[2] & 0xC0) | trimValue[1], 2);//CTR      
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[0] & 0x03) | (trimValue[2]<<2), 0);//BDC
         RF_WritePage(ROMP_REG, (A5133_RFConfigTab_Addr0x38[4] & 0x40) | trimValue[3], 4);//STM
      }
      else
         Err_State();     
   }
}
  
/*********************************************************************
** RF_RSSI_Read
*********************************************************************/
int16_t RF_RSSI_Read(void)
{
    uint8_t rssi;
   int16_t tmp;

    rssi= RF_ReadReg(RSSI_REG); //read RSSI value(wanted signal RSSI)
   
   //RF power in(dBm) = (RSSI - RL) / (RH - RL)  * 12  -  80dbm - 3
   tmp = ((rssi - Mem_RL) / (Mem_RH - Mem_RL) * 12) - 80 - 3; 
   return tmp;
}

/*********************************************************************
** RF_PM_SleepMode
*********************************************************************/
void RF_PM_SleepMode(void)
{
   RF_StrobeCmd(CMD_SLEEP);
   BOARD_A5133_SCS_H();
   //BOARD_A5133_SDIO_H();
}

/*********************************************************************
** RF_Low Voltage Reset_Check
*********************************************************************/
uint8_t RF_LVR_Check(void)
{
   uint8_t tmp;
   
   tmp = RF_ReadReg(VCOCAL2_REG);
   if(tmp == 0xFF)//default reset value 0xFF
      return 1;
   else
      return 0;
}

/*********************************************************************
** RF_WOR_En
*********************************************************************/
void RF_WOR_En(void)
{
    gio1PinCtrlReg_tu gio1PinCtrlReg;
    gio2PinCtrlReg_tu gio2PinCtrlReg;

    gio1PinCtrlReg.value = 0x00;
    gio1PinCtrlReg.bits_w.gio1oe = 1;
    gio1PinCtrlReg.bits_w.gio1s = 2;//MCU wakeup signal
   RF_WriteReg(GIO1_REG, gio1PinCtrlReg.value);
    gio2PinCtrlReg.value = 0x00;
    gio2PinCtrlReg.bits_w.gio2oe = 1;//WTR
   RF_WriteReg(GIO2_REG, gio2PinCtrlReg.value);
   RF_WriteReg(CKO_REG, 0x02);   
   
   RF_WriteReg(RCOSC1_REG, 0x0B);//WOR_SL=11, SL time=(11+1)*32*(1/4KHz)
   RF_WriteReg(RCOSC2_REG, 0x0B);//WOR_AC=11, AC time(wakeup time + RX time)=(11+1)*(1/4KHz)
   RF_WriteReg(RCOSC3_REG, 0x0C);//RCOSC enable
   
   while(1)
   { 
      RF_WriteReg(MODECTRL_REG, A5133_RFConfigTab_Main[1] | (1<<3));    //WOR enable
      while(~READ_RF_A5133_GPIO1());//waitting to wakeup until packet is received 
      
        RF_StrobeCmd(CMD_PLL);
      //Wakeup process...
      //RxPacket();
      //...;
   }
}
