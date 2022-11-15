
#include "myRadio.h"
#include "myRadio_gpio.h"
/**-------------------------radio include----------------------------------**/
#include "A5133_hal.h"
#include "A5133reg.h"
/**-------------------------radio include end----------------------------------**/

static int8_t rfTxPower;
static uint32_t rfFrequence;
static uint32_t rfBaudrate;
static rfRxCallBack rxCb;
static uint8_t rfRxBuffer[255];
static uint32_t rf_handle;
static uint8_t rf_workProcess;
static uint8_t chipType;
/**-------------------------radio params----------------------------------**/
static bool rf_irq; 
//static irqCallback_ts myIrqCallback_tim1;


rfTxPowerReg_ts txPowerList[RF_TX_PWR_MAX_COUNT] = 
{
    {.power = -30, .txlpn = 0, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 0},
    {.power = -20, .txlpn = 0, .pgv_pa = PGV_1_6V, .tbf = 6, .tpa = 1},
    {.power = -15, .txlpn = 0, .pgv_pa = PGV_1_6V, .tbf = 1, .tpa = 6},
    {.power = -10, .txlpn = 0, .pgv_pa = PGV_2_4V, .tbf = 2, .tpa = 2},
    {.power = -5, .txlpn = 0, .pgv_pa = PGV_2_4V, .tbf = 5, .tpa = 3},
    {.power = -1, .txlpn = 0, .pgv_pa = PGV_2_4V, .tbf = 7, .tpa = 6},
    {.power = 0, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 0},
    {.power = 1, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 1},
    {.power = 2, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 2},
    {.power = 3, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 4},
    {.power = 4, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 5},
    {.power = 5, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 6},
    {.power = 6, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 0, .tpa = 7},
    {.power = 7, .txlpn = 1, .pgv_pa = PGV_1_6V, .tbf = 2, .tpa = 0},
    {.power = 8, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 0, .tpa = 3},
    {.power = 9, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 0, .tpa = 4},
    {.power = 10, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 1, .tpa = 2},
    {.power = 11, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 2, .tpa = 1},
    {.power = 12, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 2, .tpa = 3},
    {.power = 13, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 1, .tpa = 7},
    {.power = 14, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 6, .tpa = 0},
    {.power = 15, .txlpn = 1, .pgv_pa = PGV_2_4V, .tbf = 7, .tpa = 7},
};
static void tim1_callback(uint8_t status, uint32_t param);
uint8_t getRfPowerTabIndex(int8_t power);
/**-------------------------radio params end----------------------------------**/
void myRadio_delay(uint32_t time_ms)
{
    uint32_t i, j;
    i = time_ms;
    while (i --)
    {
        for ( j = 0; j < 1000; j++)
        {
            ;
        }
    }
}
/**
 * @brief IO口中断回调
 *      IO口产生中断后会执行该函数
 *      用于接收射频工作的中断响应
 * 
 * @param index 
 */
void myRadio_gpioCallback(uint8_t index)
{
    if (rf_handle != 0xe5)
    {
        return;
    }
    if ((rf_workProcess != RWP_TX) && (rf_workProcess != RWP_RX))
    {
        return;
    }
    
    rf_irq = true;
	myRadio_process();
}
/**
 * @brief 射频初始化
 * 
 * @param agr0 
 * @param agr1_ptr 无线工作状态响应回调
 *          产生回调给外部使用，@rfRxCallBack
 */
void myRadio_init(int agr0, void *agr1_ptr)
{
    myRadio_gpio_init(myRadio_gpioCallback);
    
/**-------------------------radio init----------------------------------**/
    if(RF_Init()) //init RF
    {
        Err_State();
    }
    gio1PinCtrlReg_tu gio1PinCtrlReg;
    gio2PinCtrlReg_tu gio2PinCtrlReg;
    ckoPinCtrlReg_tu ckoPinCtrlReg;
    code2Page0Reg_tu code2Page0Reg;

    gio1PinCtrlReg.value = 0x00;
    gio1PinCtrlReg.bits_w.gio1oe = 1;
    gio1PinCtrlReg.bits_w.gio1s = 0;//WTR
   RF_WriteReg(GIO1_REG, gio1PinCtrlReg.value);
    gio2PinCtrlReg.value = 0x00;
    gio2PinCtrlReg.bits_w.gio2oe = 1;
    gio2PinCtrlReg.bits_w.gio2s = 7;//trxd
   RF_WriteReg(GIO2_REG, gio2PinCtrlReg.value);
    ckoPinCtrlReg.value = 0x00;
    ckoPinCtrlReg.bits_w.ckoe = 0;
    ckoPinCtrlReg.bits_w.ckos = 0;//dck
   RF_WriteReg(CKO_REG, 0x02);   
//    code2Page0Reg.value = 0;
//    code2Page0Reg.bits_w.edrl = 1;
//    code2Page0Reg.bits_w.pth = 1;
//    code2Page0Reg.bits_w.eth = 4;
//    RF_WritePage(CODE2_REG, code2Page0Reg.value, 0);   

    RF_FIFOLength(13-1);//64 bytes  
    RF_StrobeCmd(CMD_PLL);
    // RF_SetCH(80);//freq=5805.001MHz  80
    /* 
        How to set RF channel
        RF base Freq : 5725.001MHz
        Channel step : 1MHz
        SetCH        : 80
        RF Frequency = RF base Freq + (Channel step * SetCH) = 5725.001MHz + ( 1MHz * 80 ) = 5805.001MHz 
    */
/**-------------------------radio init end----------------------------------**/
    RF_EXT_PA_TO_IDLE();
    if ((rfRxCallBack )agr1_ptr)
    {
        rxCb = (rfRxCallBack )agr1_ptr;
    }
    rf_handle = 0xe5;
}
/**
 * @brief 射频底层执行程序
 *      要放在主循环中执行
 * 
 */
    rfRxPacket_ts packet;
void myRadio_process(void)
{
    if (rf_handle == 0)
    {
        return;
    }
    if (rf_irq == false)
    {
        return;
    }
    rf_irq = false;
    switch (rf_workProcess)
    {
    case RWP_TX:
    {//Tx done
        rf_workProcess = RWP_IDLE;
        rxCb(TX_STA_SECCESS, packet);
    }
        break;
    case RWP_RX:
    {//Rx done
		RF_EXT_PA_TO_IDLE();
        packet.rssi = RF_RSSI_Read();
        packet.len = 64;//RF_GetFIFOLength();
        //RF_FIFORead(packet.payload, packet.len);
        rf_workProcess = RWP_IDLE;
        rxCb(RX_STA_SECCESS, packet);
    }
        break;
    
    default:
        break;
    }
}
/**
 * @brief 退出射频进入休眠
 * 
 */
void myRadio_abort(void)
{
    if (rf_handle == 0)
    {
        return;
    }
    rf_workProcess = RWP_IDLE;
    RF_EXT_PA_TO_IDLE();
    RF_PM_SleepMode();
}
/**
 * @brief 获取射频工作中心频率
 * 
 * @return uint32_t 
 */
uint32_t myRadio_getFrequency(void)
{
    if (rf_handle == 0)
    {
        return 0;
    }
    return rfFrequence;
}
/**
 * @brief 设置射频工作中心频率
 * 
 * @param freq 
 *      具体频点，单位：Hz
 */
void myRadio_setFrequency(uint32_t freq)
{
    if (rf_handle == 0)
    {
        return;
    }
    rfFrequence = freq;
    RF_SetCH(rfFrequence);
}
/**
 * @brief 获取发射功率
 * 
 * @return int8_t 
 */
int8_t myRadio_getTxPower(void)
{
    if (rf_handle == 0)
    {
        return 0;
    }
    return rfTxPower;
}
/**
 * @brief 设置发射功率
 * @param power 
 *          单位：dbm
 */
void myRadio_setTxPower(int8_t power)
{
    txTestReg_tu txTestReg;
    code2Page8Reg_tu code2Page8Reg;
    code2Page9Reg_tu code2Page9Reg;
    code3Page1Reg_tu code3Page1Reg;
    uint8_t powerIndex;
    if (rf_handle == 0)
    {
        return;
    }
    rfTxPower = power;

    powerIndex = getRfPowerTabIndex(rfTxPower);

    txTestReg.value = RF_ReadReg(TXTEST_REG);
    txTestReg.bits_w.tbf = txPowerList[powerIndex].tbf;
    RF_WriteReg(TXTEST_REG, txTestReg.value);
    code2Page8Reg.value = RF_ReadPage(CODE2_REG, 8);
    code2Page8Reg.bits_w.tpa = txPowerList[powerIndex].tpa;
    RF_WritePage(CODE2_REG, code2Page8Reg.value, 8);
    code2Page9Reg.value = RF_ReadPage(CODE2_REG, 9);
    code2Page9Reg.bits_w.pgv_pa = txPowerList[powerIndex].pgv_pa;
    RF_WritePage(CODE2_REG, code2Page9Reg.value, 9);
    code3Page1Reg.value = RF_ReadPage(CODE3_REG, 1);
    code3Page1Reg.bits_w.txlpn = txPowerList[powerIndex].txlpn;
    RF_WritePage(CODE3_REG, code3Page1Reg.value, 1);
}
/**
 * 获取射频波特率
 * @param : br->
*/
uint32_t myRadio_getBaudrate(void)
{
    if (rf_handle == 0)
    {
        return 0;
    }
    return rfBaudrate;
}
/**
 * 设置射频波特率
 * @param : br->
*/
void myRadio_setBaudrate(uint32_t br)
{
    rfBaudrate = br;

}
/**
 * @brief 设置模组型号
 * 
 * @param type 
 */
void myRadio_setChipType(uint8_t type)
{
    chipType = type;
}
/**
 * @brief 获取模组型号
 * 
 * @return uint8_t 
 */
uint8_t myRadio_getChipType(void)
{
    return chipType;
}
int16_t myRadio_getRssi(void)
{
    return 0;
}
/**
 * @brief 无线发送数据包
 * 
 * @param packet 
 */
void myRadio_transmit(rfTxPacket_ts *packet)
{
    if (rf_handle == 0)
    {
        return;
    }
    RF_EXT_PA_TO_TX();

    if ((rf_workProcess == RWP_RX) || (rf_workProcess == RWP_SLEEP))
    {
        rf_workProcess = RWP_IDLE;
        RF_StrobeCmd(CMD_STBY);
    }
    // RF_FIFOLength(packet->len);
    RF_FIFOWrite(packet->payload, 64);  //write data to TX FIFO
    RF_StrobeCmd(CMD_TX);
    rf_workProcess = RWP_TX;
}
/**
 * @brief 进入无线接收
 * 
 */
void myRadio_receiver(void)
{
    if (rf_handle == 0)
    {
        return;
    }

    if ((rf_workProcess == RWP_TX) || (rf_workProcess == RWP_SLEEP))
    {
        rf_workProcess = RWP_IDLE;
    }
    RF_StrobeCmd(CMD_RX);
    RF_EXT_PA_TO_RX();
	tst();

    rf_workProcess = RWP_RX;
}
void myRadio_setCtrl(controlMode_te mode, uint32_t value)
{
    tx1Reg_tu tx1Reg;
    if (rf_handle == 0)
    {
        return;
    }
    myRadio_init(0, 0);
    RF_SetCH(rfFrequence);
    switch (mode)
    {
    case RADIO_EXT_CONTROL_TX_UNMODULATED:
    {

        RF_EXT_PA_TO_TX();
        myRadio_setTxPower(rfTxPower);
        RF_StrobeCmd(CMD_TFR);//TX fifo pointer reset
        tx1Reg.value = RF_ReadReg(TX1_REG);
        tx1Reg.bits_w.tmde = 0;
        tx1Reg.bits_w.tme = 0;
        RF_WriteReg(TX1_REG, tx1Reg.value);
        RF_StrobeCmd(CMD_TX);
    }
        break;
    case RADIO_EXT_CONTROL_TX_MODULATED:
    {
        RF_EXT_PA_TO_TX();
        RF_StrobeCmd(CMD_TFR);//TX fifo pointer reset
        myRadio_setTxPower(rfTxPower);
        tx1Reg.value = RF_ReadReg(TX1_REG);
        tx1Reg.bits_w.tmde = 1;
        tx1Reg.bits_w.tme = 1;
        RF_WriteReg(TX1_REG, tx1Reg.value);
        RF_StrobeCmd(CMD_TX);
    }
        break;
    case RADIO_EXT_CONTROL_RX_SENSITIVITY:
    {
        RF_EXT_PA_TO_RX();
        // pageA_gio_tu pageA_gio;

        // pageA_gio.value = 0;
        // pageA_gio.bits_w.g1oe = 1;
        // pageA_gio.bits_w.gio1s = GIOMD_EOAC_FSYNC;
        // A7169_WritePageA(GIO_PAGEA, pageA_gio.value);    //GIO1=TRXD
        // A7169_WritePageA(CKO_PAGEA, 0xD28B);    //CKO=RCK
        // base_modeControl_tu base_modeControl;
        // base_modeControl.value = 0;
        // base_modeControl.bits_w.fms = 0;
        // base_modeControl.bits_w.trsr = 0;
        // base_modeControl.bits_w.cer = 1;
        // base_modeControl.bits_w.cce = 1;
        // A7169_WriteReg(MODE_REG, base_modeControl.value);   //set FMS=0, Direct mode
                    
        // RF_StrobeCmd(CMD_RX);  //
        // myRadio_delay(10);
        // pageA_gio.value = 0;
        // pageA_gio.bits_w.g1oe = 1;
        // pageA_gio.bits_w.gio1s = GIOMD_RXD;
        // A7169_WritePageA(GIO_PAGEA, pageA_gio.value);    //GIO1=TRXD
    }
        break;
    
    default:
        break;
    }
}

/**-------------------------radio funtion----------------------------------**/
static void tim1_callback(uint8_t status, uint32_t param)
{
    if (rand()&0x01)
    {
        RF_A5133_GPIO1_H();
    }
    else
    {
        RF_A5133_GPIO1_L();
    }
}
uint8_t getRfPowerTabIndex(int8_t power)
{
    for (int i = 0; i < sizeof(txPowerList)/sizeof(rfTxPowerReg_ts); i++)
    {
        if (txPowerList[i].power >= power)
        {
            return i;
        }
    }
    return sizeof(txPowerList)/sizeof(rfTxPowerReg_ts) - 1;
}
/**-------------------------radio funtion end----------------------------------**/
