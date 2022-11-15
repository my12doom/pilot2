/********************************************************************
*   A5133REG.h
*   RF Chip-A5133 Hardware Definitions
*
*   This file provides the constants associated with the
*   AMICCOM A5133 device.
*
********************************************************************/
#ifndef _A5133REG_h_
#define _A5133REG_h_
#include <stdint.h>

#define MODE_REG 				0x00
#define MODECTRL_REG		   0x01
#define CALIBRATION_REG	   0x02
#define FIFO1_REG 			0x03
#define FIFO2_REG 			0x04
#define FIFO_REG 				0x05
#define IDCODE_REG 			0x06
#define RCOSC1_REG 			0x07
#define RCOSC2_REG 			0x08
#define RCOSC3_REG 			0x09
#define CKO_REG 				0x0A
#define GIO1_REG 				0x0B
#define GIO2_REG 				0x0C
#define DATARATE_REG 		0x0D
#define PLL1_REG 				0x0E
#define PLL2_REG				0x0F
#define PLL3_REG 				0x10
#define PLL4_REG				0x11
#define PLL5_REG 				0x12
#define CHGROUP1_REG		0x13
#define CHGROUP2_REG		0x14
#define TX1_REG  				0x15
#define TX2_REG  				0x16
#define DELAY1_REG			0x17
#define DELAY2_REG			0x18
#define RX_REG					0x19
#define RXGAIN1_REG			0x1A
#define RXGAIN2_REG			0x1B
#define RXGAIN3_REG			0x1C
#define RXGAIN4_REG			0x1D
#define RSSI_REG				0x1E
#define ADC_REG  				0x1F
#define CODE1_REG 			0x20
#define CODE2_REG 			0x21
#define CODE3_REG 			0x22
#define IFCAL1_REG  		0x23
#define IFCAL2_REG  		0x24
#define VCOCCAL_REG  		0x25
#define VCOCAL1_REG  		0x26
#define VCOCAL2_REG  		0x27
#define VCODEVCAL1_REG  0x28
#define VCODEVCAL2_REG  0x29
#define DASP_REG 	 			0x2A
#define VCOMODDELAY_REG	0x2B
#define BATTERY_REG  		0x2C
#define TXTEST_REG  		0x2D
#define RXDEM1_REG  		0x2E
#define RXDEM2_REG  		0x2F
#define CPC1_REG				0x30
#define CPC2_REG				0x31
#define CRYSTALTEST_REG	0x32
#define PLLTEST_REG   	0x33
#define VCOTEST_REG 		0x34
#define RFANALOG_REG 		0x35
#define KEYDATA_REG 		0x36
#define CHSELECT_REG		0x37
#define ROMP_REG 				0x38
#define DATARATECLOCK		0x39
#define FCR_REG 				0x3A
#define ARD_REG 				0x3B
#define AFEP_REG 				0x3C
#define FCB_REG 				0x3D
#define KEYC_REG 				0x3E
#define USID_REG 				0x3F

//strobe command
#define CMD_SLEEP		      0x80	//1000,xxxx	SLEEP mode
#define CMD_IDLE		      0x90	//1001,xxxx	IDLE mode
#define CMD_STBY		      0xA0	//1010,xxxx Standby mode
#define CMD_PLL			   0xB0	//1011,xxxx	PLL mode
#define CMD_RX			      0xC0	//1100,xxxx	RX mode
#define CMD_TX			      0xD0	//1101,xxxx	TX mode
#define CMD_TFR			   0xE0	//1110,xxxx	TX FIFO reset
#define CMD_RFR			   0xF0	//1111,xxxx	RX FIFO reset
#define CMD_DPSLEEP_TRI		0x88	//1000,1000	deep sleep tri mode
#define CMD_DPSLEEP_PULL   0x8B	//1000,1011 deep sleep Pull high mode

//@MODE_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t resetn	: 8;    //Write to this register by 0x00 to issue reset command, then it is auto clear
    }bits_w;
    struct 
    {
        uint8_t trer    : 1;    //: TRX Enable Register.
                                // [0]: Disable.
                                // [1]: Enable. It will be clear after end of packet encountered in FIFO mode.
        uint8_t trsr    : 1;    //TRX Mode Select Register.
                                // [0]: RX.
                                // [1]: TX. When TRE set, the chip will enter TX or RX mode by TRS register.
        uint8_t pller    : 1;    //PLL enable Register.
                                // [0]: PLL is disabled.
                                // [1]: PLL is enabled.
        uint8_t xer    : 1;    //: Internal crystal oscillator enable Register.
                                // [0]: Crystal oscillator is disabled.
                                // [1]: Crystal oscillator is enabled
        uint8_t cer    : 1;    //RF chip enable Register.
                                // [0]: RF chip is disabled.
                                // [1]: RF chip is enabled.
        uint8_t crcf    : 1;    //CRC flag. (CRCF is read clear.)
                                // [0]: CRC pass.
                                // [1]: CRC error
        uint8_t fecf    : 1;    //FEC flag. (FECF is read clear.)
                                // [0]: FEC pass.
                                // [1]: FEC error
        uint8_t hecf    : 1;    //Head Control Flag. (Clear by any Strobe command.)
                                // HEC is CRC-8 result from FCB + DFL.
                                // [0]: HEC pass.
                                // [1]: HEC error.
    }bits_r;
}modeReg_tu;
//@MODECTRL_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t adcm	: 1;    //ADC measurement enables (Auto clear when done).
                                // [0]: Disable measurement or measurement finished.
                                // [1]: Enable measurement.
        uint8_t fms     : 1;    //Direct/FIFO mode select.
                                // [0]: Direct mode.
                                // [1]: FIFO mode.
        uint8_t fmt     : 1;    //Reserved for internal usage only
        uint8_t wore    : 1;    //Wake On RX enable.
                                // [0]: Disable.
                                // [1]: Enable.
        uint8_t dfcd	: 1;    //: DFCD: Data Filter by CD.
                                // [0]: Disable.
                                // [1]: Enable. The data package would be filtered while the input power level is below the threshold level (RTH[7:0], 1Eh).
                                // DFCD (Read only): Carrier detector signal.
                                // [0]: Input power below threshold.
                                // [1]: Input power above threshold.
        uint8_t aif     : 1;    //(Auto IF Offset): RF LO frequency will auto offset one IF frequency while entering RX mode.
                                // [0]: Disable.
                                // [1]: Enable.
                                // If AIF =1, then,
                                // FRXLO = FPLLS - FIF, for up side band (ULS = 0, 19h).
                                // FRXLO = FPLLS + FIF, for low side band (ULS = 1, 19h)
        uint8_t arssi   : 1;    // Auto RSSI measurement while entering RX mode. Recommend ARSSI = [1].
                                // [0]: Disable.
                                // [1]: Enable.
        uint8_t ddpc    : 1;    //(Direct mode data pin control): Direct mode modem data can be accessed via SDIO pin when this register is enabled.
                                // [0]: Disable.
                                // [1]: Enable
    }bits_w;
    struct 
    {
        uint8_t adcm    : 1;    //
        uint8_t fms    : 1;    //
        uint8_t fmt    : 1;    //
        uint8_t wore    : 1;    //
        uint8_t cd    : 1;    //
        uint8_t aif    : 1;    //
        uint8_t arssi    : 1;    //
        uint8_t ddpc    : 1;    //
    }bits_r;
}modeCtrlReg_tu;
//@FIFO2_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t psa    : 6;    //Used for Segment FIFO
        uint8_t fpm    : 2;    //FIFO Pointer Margin
                                // [00]: 4 bytes.
                                // [01]: 8 bytes.
                                // [10]: 12 bytes.
                                // [11]: 16 bytes.
    }bits_w;
    struct 
    {
        uint8_t fifopt    : 8;    //FIFO pointer index (read only).
                                    // The FIFO access pointer = FIFOPT x 2.
    }bits_r;
}fifo2Reg_tu;
//@FIFO_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t fifo    : 8;    //FIFO data.
                                // TX FIFO and RX FIFO share the same address (05h).
                                // TX FIFO is max 64-byte write only.
                                // RX FIFO is max 64-byte read only.
    }bits_w;
    struct 
    {
        uint8_t fifo    : 8;    //FIFO data.
                                // TX FIFO and RX FIFO share the same address (05h).
                                // TX FIFO is max 64-byte write only.
                                // RX FIFO is max 64-byte read only.
    }bits_r;
}fifoReg_tu;
//@IDCODE_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t id    : 8;    //: ID data (sync word, max 8 bytes).
                            // When this address is accessed, ID Data is input or output sequential (ID Byte 0,1, 2, 3 …., 7) corresponding to Write or Read
    }bits_w;
    struct 
    {
        uint8_t id    : 8;    //: ID data (sync word, max 8 bytes).
                                // When this address is accessed, ID Data is input or output sequential (ID Byte 0,1, 2, 3 …., 7) corresponding to Write or Read
    }bits_r;
}idDataReg_tu;
//@RCOSC1_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t wor_sl0_7    : 8;    //
    }bits_w;
}rcOsc1Reg_tu;
//@RCOSC2_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t wor_ac    : 6;    //: 6-bits WOR Active Timer for TWOR Function
        uint8_t wor_sl8_9    : 2;    //10-bits WOR Sleep Timer for TWOR Function.
                                    // WOR_SL [9:0] are from address (07h) and (08h),
                                    // Device Active = (WOR_AC+1) x (1/4000), (250us ~ 16ms).
                                    // Device Sleep = (WOR_SL+1) x (1/4000) x 32, (8ms ~ 8.192s)
    }bits_w;
}rcOsc2Reg_tu;
//@RCOSC3_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t twor_e    : 1;    //Enable TWOR function.
                                // [0]: Disable TWOR function.
                                // [1]: Enable TWOR mode. Wake up MCU by a periodic TWOR output
        uint8_t tsel    : 1;    //Timer select for TWOR function.
                                // [0]: Use WOR_AC.
                                // [1]: Use WOR_SL.
        uint8_t rcosc_e    : 1;    //RC Oscillator Enable.
                                    // [0]: Disable.
                                    // [1]: Enable.
        uint8_t rcks    : 2;    //RO calibration clock select:
                                // [00]: 32XDR
                                // [01]: 16MHz
                                // [1x]: 8XDR
        uint8_t mrc    : 1;    //Manual RC Bank value setting.
                                // [0]: Auto.
                                // [1]: Manual.
        uint8_t irchc    : 1;    //Ring oscillator high current mode select
        uint8_t resv    : 1;    //--
    }bits_w;
}rcOsc3Reg_tu;
//@CKO_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t scki    : 1;    //SPI clock input invert.
                                // [0]: Non-inverted input.
                                // [1]: Inverted input.
        uint8_t ckoe    : 1;    //CKO pin Output Enable.
                                // [0]: High Z.
                                // [1]: Enable
        uint8_t ckoi    : 1;    //CKO pin output signal invert.
                                // [0]: Non-inverted output.
                                // [1]: Inverted output.
        uint8_t ckos    : 4;    //: CKO pin output select.
                                // [0000]: DCK (TX data clock).
                                // [0001]: RCK (RX recovery clock).
                                // [0010]: FPF (FIFO pointer flag).
                                // [0011]: Logic OR gate by EOP, EOVBC, EOFBC, EOVCC, EOVDC, RSSC_OK and inverter signal of X’tal ready. (Internal
                                // usage only).
                                // [0100]: FSYCK / 2.
                                // [0101]: FSYCK / 4.
                                // [0110]: RXD.
                                // [0111]: BOD.
                                // [1000]: WCK.
                                // [1001]: FSYNC.
                                // [1010]: ROSC.
                                // [1011]: MXDEC (MXT=0:inverter signal of OKADCN, MXT=1: DEC)
                                // [1100]: BDF.
                                // [1101]: FSYCK .
                                // [1110]: VPOAK
                                // [1111]: WRTC
        uint8_t eckoe    : 1;    //External Clock Output Enable for CKOS [3:0]= [0100] ~ [0111].
                                // [0]: Disable.
                                // [1]: Enable.
    }bits_w;
}ckoPinCtrlReg_tu;
//@GIO1_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t gio1oe    : 1;    //GIO1pin output enable.
                                // [0]: High Z.
                                // [1]: Enable
        uint8_t gio1i    : 1;    //GIO1 pin output signal invert.
                                // [0]: Non-inverted output.
                                // [1]: Inverted output.
        uint8_t gio1s    : 4;    // GIO1 pin function select.
                                // GIO1S [3:0]      TX state                |          RX state
                                //------------------------------------------|------------------------------
                                // [0000]       WTR (Wait until TX or RX finished)
                                // [0001]       EOAC (end of access code)   |           FSYNC
                                // [0010]       TMEO or TMDEO(TX            |           CD(carrier detect)
                                //                      modulation enable)
                                // [0011]               SID1 Detect Output(ID1DO)
                                // [0100]               RCOSC_E=1: MCU wakeup signal (TWOR);
                                //                      RCOSC_E=0: CWTR
                                // [0101]               MTCRCINT / VTB0 /In phase demodulator input(DMII)
                                // [0110]               SDO ( 4 wires SPI data out)
                                // [0111]               TRXD In/Out ( Direct mode )
                                // [1000]               RXD ( Direct mode )
                                // [1001]               TXD ( Direct mode )
                                // [1010]               PDN_RX
                                // [1011]               External FSYNC input in RX direct mode *
                                // [1100]               MXINC(MXT=0:EOADC.MXT=1:INC.)
                                // [1101]               FPF
                                // [1110]               VPOAK (Auto Resend OK Output)
                                // [1111]               FMTDO (FIFO mode TX Data Output testing)
        uint8_t vpm    : 1;    //Valid Pulse width select.
                                // [0]: 10us.
                                // [1]: 30us.
        uint8_t vkm    : 1;    //Valid packet mode select.
                                // [0]: by event.
                                // [1]: by pulse.
    }bits_w;
}gio1PinCtrlReg_tu;
//@GIO2_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t gio2oe    : 1;    //GIO1pin output enable.
                                // [0]: High Z.
                                // [1]: Enable.
        uint8_t gio2i    : 1;    //GIO2 pin output signal invert.
                                // [0]: Non-inverted output.
                                // [1]: Inverted output.
        uint8_t gio2s    : 4;    //GIO2 pin function select.
                                // GIO2S [3:0]          TX state            |               RX state
                                //------------------------------------------|------------------------------
                                // [0000]               WTR (Wait until TX or RX finished)
                                // [0001]       EOAC (end of access code)   |        FSYNC(frame sync)
                                // [0010] TMEO(TX modulation enable)        |        CD(carrier detect)
                                // [0011]           SID1 Detect Output (ID1DO
                                // [0100]           RCOSC_E=1: MCU wakeup signal (TWOR);
                                //                    RCOSC_E=0: CWTR
                                // [0101]           MTCRCINT/ VTB1 /Quadrature phase demodulator output (DMIQ).
                                // [0110]           SDO ( 4 wires SPI data out)
                                // [0111]           TRXD In/Out ( Direct mode )
                                // [1000]           RXD ( Direct mode )
                                // [1001]           TXD ( Direct mode )
                                // [1010]           PDN_TX
                                // [1011]
                                // [1100]           BDF
                                // [1101]           FPF
                                // [1110]           VPOAK (Auto Resend OK Output)
                                // [1111]           FMTCK (FIFO mode TX Data clock Output testing)
        uint8_t bbcks    : 2;    //Clock s          elect for digital block.
                                // [00]: F          SYCK
                                // [01]: F          SYCK / 2.
                                // [10]: F          SYCK / 4.
                                // [11]: F          SYCK / 8.
                                // FSYCK is A5133’s System clock = 16MHz
    }bits_w;
}gio2PinCtrlReg_tu;
//@TX1_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t fdp    : 3;    //Frequency deviation power setting.
        uint8_t tme    : 1;    //TX modulation enable. Recommend TME = [1].
                                // [0]: Disable.
                                // [1]: Enable.
        uint8_t txdi    : 1;    //TX data invert. Recommend TXDI = [0].
                                // [0]: Non-invert.
                                // [1]: Invert.
        uint8_t tmde    : 1;    //TX Modulation Enable for VCO Modulation. Recommend TMDE = [1].
                                // [0]: Disable.
                                // [1]: Enable.
        uint8_t gf    : 1;    //Gaussian Filter Over-sampling Rate Select. Recommend GDR = [0].
        uint8_t gdr    : 1;    // GF: Gaussian Filter Select.
                                // [0]: Disable.
                                // [1]: Enable.
    }bits_w;
}tx1Reg_tu;
//@RSSI_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t rth    : 8;    //: Carrier detect threshold.
                                // CD (Carrier Detect) =1 when RSSI ≧ RTH.
                                // CD (Carrier Detect) =0 when RSSI < RTL
    }bits_w;
    struct 
    {
        uint8_t adc    : 8;    //ADC output value of thermal sensor and RSSI (read only).
                                // ADC input voltage= 0.6 * ADC [7:0] / 256 V
    }bits_r;
}rssiThresHoldReg_tu;
//@CODE1_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t epml    : 2;    //Extend Preamble Length Select. Recommend EPML= [00].
                                // [00]: 0 byte.
                                // [01]: 1 byte.
                                // [10]: 2 bytes.
                                // [11]: 4 bytes
        uint8_t idl    : 2;    //: ID Code Length Select. Recommend IDL= [11].
                                // [00]: Reserved.
                                // [01]: 4 bytes.
                                // [10]: Reserved.
                                // [11]: 8 bytes.
                                // If user selects 4Bytes ID code, it is called SID1. If user selects 8Bytes ID code, the first 4Bytes ID code is called SID1 and the
                                // second 4Bytes ID code is called SID2.
        uint8_t crcs    : 1;    //CRC Select.
                                // [0]: Disable.
                                // [1]: Enable. The CRC is set by CRCDNP (0x1A) for either CCITT-16 CRC or CRC-DNP
        uint8_t fecs    : 1;    //FEC Select.
                                // [0]: Disable.
                                // [1]: Enable (The FEC is (7, 4) Hamming code)
        uint8_t whts    : 1;    //Data Whitening (Data Encryption) Select.
                                // [0]: Disable.
                                // [1]: Enable (The data is whitening by multiplying PN7)
        uint8_t resv    : 1;    //--
    }bits_w;
    struct 
    {
        uint8_t snf    : 8;    //Sub-package Flag (read only).
    }bits_r;
}code1Reg_tu;
//@CODE2_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t pth    : 2;    //Received SID1 Code Error Tolerance.
                                // [00]: 0 bit,
                                // [01]: 1 bit.
                                // [10]: 2 bit.
                                // [11]: 3 b
        uint8_t eth    : 3;    //Received SID2 Code Error Tolerance. SID2 is only valid if ID length is 8bytes.
                                // [000]: 0 bit,
                                // [001]: 1 bit.
                                // [010]: 2 bit.
                                // [011]: 3 bit.
                                // [100]: 4 bit,
                                // [101]: 5 bit.
                                // [110]: 6 bit.
                                // [111]: 7 bit
        uint8_t hecs    : 1;    //Head CRC Select
                                // [0]: disable.
                                // [1]: enable
        uint8_t edrl    : 1;    //Enable FIFO Dynamic Length
                                // [0]: Disable.
                                // [1]: Enable
        uint8_t mscrc    : 1;    //Mask CRC (CRC Data Filtering Enable).
                                // [0]: Disable.
                                // [1]: Enable.
    }bits_w;
    struct 
    {
        uint8_t mtcrcf0_7    : 8;    //Sub-package CRC Flag (read only).
    }bits_r;
}code2Page0Reg_tu;
//@CODE2_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t pfine    : 3;    //RC-OSC fine tuned value
        uint8_t resv1    : 1;    //
        uint8_t tpa    : 3;    //PA current setting
        uint8_t resv2    : 1;    //
    }bits_w;
}code2Page8Reg_tu;
typedef enum
{
    PGV_1_2V,
    PGV_1_4V,
    PGV_1_6V,
    PGV_1_8V,
    PGV_2_0V,
    PGV_2_2V,
    PGV_2_4V,
    PGV_2_5V,
}GPV_te;
//@CODE2_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t pgv_pa    : 3;    //Power gain voltage for PA, @GPV_te
        uint8_t pm1sw    : 2;    //PM1 switch select
        uint8_t pm1swen    : 1;    //PM1 switch enable
        uint8_t pmpar    : 1;    //PM1 switch gating
        uint8_t porips    : 1;    //Reserved for internal usage
    }bits_w;
}code2Page9Reg_tu;
//@CODE3_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t ws    : 7;    //Data Whitening Seed (data encryption key).
        uint8_t crcinv    : 1;    //CRC Inverted Select.
                                // [0]: Non-inverted.
                                // [1]: inverted.
    }bits_w;
    struct 
    {
        uint8_t mtcrcf8_15    : 8;    //Sub-package CRC Flag (read only).
    }bits_r;
}code3Page0Reg_tu;
//@CODE3_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t xrcs    : 1;    //
        uint8_t psdpas    : 3;    //power saving signal delay select for PSNPA
        uint8_t txlpn    : 1;    //TX low power select
        uint8_t bias    : 1;    //
        uint8_t bgcs    : 1;    //
        uint8_t txhp    : 1;    //TX high power select
    }bits_w;
}code3Page1Reg_tu;
//@IFCAL1_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t mfb    : 4;    //
        uint8_t mfbs    : 1;    //
        uint8_t ckgs    : 2;    //
        uint8_t hfr    : 1;    //
    }bits_rw;
}ifcal1Reg_tu;
//@BATTERY_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t bd_e    : 1;    //Battery Detect Enable.
                                // [0]: Disable.
                                // [1]: Enable. 
        uint8_t bvt    : 3;    //Battery Voltage Threshold Select.
                                // [000]: 2.0V,
                                // [001]: 2.1V.
                                // [010]: 2.2V.
                                // [011]: 2.3V.
                                // [100]: 2.4V.
                                // [101]: 2.5V.
                                // [110]: 2.6V.
                                // [111]: 2.7V.
        uint8_t qds    : 1;    //VDD_A Quick Discharge Select. Recommend QDS = [1].
                                // [0]: Disable.
                                // [1]: Enable.
        uint8_t bgs    : 1;    //Bangap (BG) select:
                            // [0]: Low current BG.
                            // [1]: High current BG.
                            // Sleep mode should be set to [0]
        uint8_t pm1s    : 1;    //PM1 select.
                                // [0]: Disable.
                                // [1]: Enable
        uint8_t resv    : 1;    //--
    }bits_w;
    struct 
    {
        uint8_t bd_e    : 1;    //Battery Detect Enable.
                                // [0]: Disable.
                                // [1]: Enable. 
        uint8_t bvt    : 3;    //Battery Voltage Threshold Select.
                                // [000]: 2.0V,
                                // [001]: 2.1V.
                                // [010]: 2.2V.
                                // [011]: 2.3V.
                                // [100]: 2.4V.
                                // [101]: 2.5V.
                                // [110]: 2.6V.
                                // [111]: 2.7V.
        uint8_t bdf    : 1;    //: Low Battery Detection Flag (read only).
                                // [0]: battery low.
                                // [1]: battery high.
        uint8_t resv    : 3;    //--
    }bits_r;
}batteryReg_tu;
//@ROMP_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t pd_bod    : 1;    //BOD circuit power down.
                                // [0]: Power on.
                                // [1]: Power down
        uint8_t regcl    : 1;    //Reserved for internal usage
        uint8_t bdc    : 6;    //Battery detector current option select
    }bits_w;
}romp0Reg_tu;
//@FCR_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t ear    : 1;    //Enable auto-resend.
                            // [0]: Disable.
                            // [1]: Enable
        uint8_t eak    : 1;    //Enable auto-ack.
                            // [0]: Disable.
                            // [1]: Enable.
        uint8_t arc    : 4;    //Decremented ARC[3:0] (read only).
        uint8_t fcl    : 2;    //Frame Control Length.
                            // [00]: No Frame Control
                            // [01]: 1 byte Frame Control. (FCB0)
                            // [10]: 2 byte Frame control. (FCB0+FCB1)
                            // [11]: 4 byte Frame control. (FCB0+FCB1+FCB2+FCB3)
    }bits_w;
    struct 
    {
        uint8_t ear    : 1;    //Enable auto-resend.
                            // [0]: Disable.
                            // [1]: Enable
        uint8_t eak    : 1;    //Enable auto-ack.
                            // [0]: Disable.
                            // [1]: Enable.
        uint8_t rcr    : 4;    //Auto Resend Cycle Setting.
                                // [0000]: resend disable.
                                // [0001]: 1
                                // [0010]: 2
                                // [0011]: 3
                                // [0100]: 4
                                // [0101]: 5
                                // [0110]: 6
                                // [0111]: 7
                                // [1000]: 8
                                // [1001]: 9
                                // [1010]: 10
                                // [1011]: 11
                                // [1100]: 12
                                // [1101]: 13
                                // [1110]: 14
                                // [1111]: 15
        uint8_t vpoak    : 1;    //Valid Packet or ACK OK Flag. (read only)
                                // This bit is clear by any Strobe command.
                                // [0]: Neither valid packet nor ACK OK.
                                // [1]: Valid packet or ACK OK
        uint8_t artef    : 1;    //Auto re-transmission ending flag (read only).
                                // [0]: Resend not end
                                // [1]: Finish resend.
    }bits_r;
}fcrReg_tu;
//@AFEP_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t ackfep    : 6;    //: FIFO Length setting for auto-ack packet.
                                // ACK FIFO Length = (ACKFEP [5:0] + 1)
                                // max. 64 bytes
        uint8_t spss    : 1;    //Mode Back Select when auto-act and auto-resend are enabled.
                                // [0]: Reserved.
                                // [1]: PLL mode
        uint8_t eaf    : 1;    //Enable ACK FIFO.
                            // [0]: Disable.
                            // [1]: Enable.
    }bits_w;
    struct 
    {
        uint8_t txsn    : 3;    //TX Serial Number.
                                // acket and keep the same TXSN when retransmitting
        uint8_t earts    : 3;    //Enable Auto Resend Read.
        uint8_t resv    : 3;    //--
    }bits_r;
}afepReg_tu;
//@TXTEST_REG
typedef union 
{
    uint8_t value;
    struct 
    {
        uint8_t tbf    : 3;    //TX Buffer Setting.
                                // Refer to A5133 App. Note for more settings
        uint8_t resv    : 2;    //--
        uint8_t txcs    : 1;    //TX current setting.
        uint8_t asmv    : 2;    //Ramp up/down clock select.
                                // [00]: 1 MHz.
                                // [01]: 1/2 MHz.
                                // [10]: 1/4 MHz.
                                // [11]: 1/8 MHz.
    }bits_w;
}txTestReg_tu;
#endif
