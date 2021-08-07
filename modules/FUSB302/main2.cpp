#include <stdint.h>
#include "oled.h"
#include <stm32f0xx.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/Interface/II2C.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <stdio.h>
#include "reg.h"
#include <stdarg.h>

extern "C"
{
#include <modules/utils/SEGGER_RTT.h>
}

using namespace STM32F0;
using namespace HAL;
static const uint8_t fusb302_address = 0x44;


F0GPIO scl(GPIOA, GPIO_Pin_1);
F0GPIO sda(GPIOA, GPIO_Pin_0);
F0GPIO INT(GPIOA, GPIO_Pin_2);
F0GPIO led(GPIOA, GPIO_Pin_9);
I2C_SW i2c(&scl, &sda);

bool vconn_enabled = false;
int cc_polarity = 0;
int got_int = 0;

uint16_t xy(uint8_t x, uint8_t y)
{
	return (x<<8) | y;
}

int set_polarity(int polarity)
{
   /* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
    uint8_t reg;

    i2c.read_reg(fusb302_address, TCPC_REG_SWITCHES0, &reg);

    /* clear VCONN switch bits */
    reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
    reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

    if (vconn_enabled) {
        /* set VCONN switch to be non-CC line */
        if (polarity)
            reg |= TCPC_REG_SWITCHES0_VCONN_CC1;
        else
            reg |= TCPC_REG_SWITCHES0_VCONN_CC2;
    }

    /* clear meas_cc bits (RX line select) */
    reg &= ~TCPC_REG_SWITCHES0_MEAS_CC1;
    reg &= ~TCPC_REG_SWITCHES0_MEAS_CC2;

    /* set rx polarity */
    if (polarity)
        reg |= TCPC_REG_SWITCHES0_MEAS_CC2;
    else
        reg |= TCPC_REG_SWITCHES0_MEAS_CC1;

    i2c.write_reg(fusb302_address, TCPC_REG_SWITCHES0, reg);

    i2c.read_reg(fusb302_address, TCPC_REG_SWITCHES1, &reg);

    /* clear tx_cc bits */
    reg &= ~TCPC_REG_SWITCHES1_TXCC1_EN;
    reg &= ~TCPC_REG_SWITCHES1_TXCC2_EN;

    /* set tx polarity */
    if (polarity)
        reg |= TCPC_REG_SWITCHES1_TXCC2_EN;
    else
        reg |= TCPC_REG_SWITCHES1_TXCC1_EN;

    // Enable auto GoodCRC sending
    reg |= TCPC_REG_SWITCHES1_AUTO_GCRC;

    i2c.write_reg(fusb302_address, TCPC_REG_SWITCHES1, reg);

    /* Save the polarity for later */
    cc_polarity = polarity;

	return 0;
}

int set_vconn(int enable)
{
    uint8_t reg;

    /* save enable state for later use */
    vconn_enabled = enable;

    if (enable) {
        /* set to saved polarity */
        set_polarity(cc_polarity);
    } else {

        i2c.read_reg(fusb302_address, TCPC_REG_SWITCHES0, &reg);

        /* clear VCONN switch bits */
        reg &= ~TCPC_REG_SWITCHES0_VCONN_CC1;
        reg &= ~TCPC_REG_SWITCHES0_VCONN_CC2;

        i2c.write_reg(fusb302_address, TCPC_REG_SWITCHES0, reg);
    }

	return 0;
}

int fusb302_init()
{
    uint8_t reg;
    // Restore default settings 
    i2c.write_reg(fusb302_address, TCPC_REG_RESET, TCPC_REG_RESET_SW_RESET);
	
    // Turn on retries and set number of retries
    i2c.read_reg(fusb302_address, TCPC_REG_CONTROL3, &reg);
	
    reg |= TCPC_REG_CONTROL3_AUTO_RETRY;
    reg |= (PD_RETRY_COUNT & 0x3) << TCPC_REG_CONTROL3_N_RETRIES_POS;
    i2c.write_reg(fusb302_address, TCPC_REG_CONTROL3, reg);

    // Create interrupt masks 
    reg = 0xFF;
    // CC level changes 
    reg &= ~TCPC_REG_MASK_BC_LVL;
    // collisions 
    reg &= ~TCPC_REG_MASK_COLLISION;
    // misc alert 
    reg &= ~TCPC_REG_MASK_ALERT;
    i2c.write_reg(fusb302_address, TCPC_REG_MASK, reg);

    reg = 0xFF;
    // when all pd message retries fail... 
    reg &= ~TCPC_REG_MASKA_RETRYFAIL;
    // when fusb302 send a hard reset. 
    reg &= ~TCPC_REG_MASKA_HARDSENT;
    // when fusb302 receives GoodCRC ack for a pd message 
    reg &= ~TCPC_REG_MASKA_TX_SUCCESS;
    // when fusb302 receives a hard reset 
    reg &= ~TCPC_REG_MASKA_HARDRESET;
    i2c.write_reg(fusb302_address, TCPC_REG_MASKA, reg);

    reg = 0xFF;
    // when fusb302 sends GoodCRC to ack a pd message 
    reg &= ~TCPC_REG_MASKB_GCRCSENT;
    i2c.write_reg(fusb302_address, TCPC_REG_MASKB, reg);

    // Interrupt Enable 
    i2c.read_reg(fusb302_address, TCPC_REG_CONTROL0, &reg);
    reg &= ~TCPC_REG_CONTROL0_INT_MASK;
    i2c.write_reg(fusb302_address, TCPC_REG_CONTROL0, reg);

    // Set VCONN switch defaults 
    //set_polarity(0);
    set_vconn(0);

    // Turn on the power! 
    // TODO: Reduce power consumption 
    i2c.write_reg(fusb302_address, TCPC_REG_POWER, TCPC_REG_POWER_PWR_ALL);
	
	return 0;
}

uint8_t getReg(uint8_t reg)
{
	uint8_t v;
	i2c.read_reg(fusb302_address, reg, &v);
	return v;
}
void setReg(uint8_t reg, uint8_t v)
{
	i2c.write_reg(fusb302_address, reg, v);
}
void sendPacket(
      uint8_t num_data_objects,
      uint8_t message_id,
      uint8_t port_power_role,
      uint8_t spec_rev,
      uint8_t port_data_role,
      uint8_t message_type,
      uint8_t *data_objects )
{
	uint8_t temp;
	uint8_t tx_buf[40];

	tx_buf[0]  = 0x12; // SOP, see USB-PD2.0 page 108
	tx_buf[1]  = 0x12;
	tx_buf[2]  = 0x12;
	tx_buf[3]  = 0x13;
	tx_buf[4]  = (0x80 | (2 + (4*(num_data_objects & 0x1F))));
	tx_buf[5]  = (message_type & 0x0F);
	tx_buf[5] |= ((port_data_role & 0x01) << 5);
	tx_buf[5] |= ((spec_rev & 0x03) << 6);
	tx_buf[6]  = (port_power_role & 0x01);
	tx_buf[6] |= ((message_id & 0x07) << 1);
	tx_buf[6] |= ((num_data_objects & 0x07) << 4);

	//Serial.print("Sending Header: 0x");
	//Serial.println(*(int *)(tx_buf+5), HEX);
	//Serial.println();

	temp = 7;
	for(uint8_t i=0; i<num_data_objects; i++) {
	tx_buf[temp]   = data_objects[(4*i)];
	tx_buf[temp+1] = data_objects[(4*i)+1];
	tx_buf[temp+2] = data_objects[(4*i)+2];
	tx_buf[temp+3] = data_objects[(4*i)+3];
	temp += 4;
	}

	tx_buf[temp] = 0xFF; // CRC
	tx_buf[temp+1] = 0x14; // EOP
	tx_buf[temp+2] = 0xFE; // TXOFF
	/*
	Serial.print("Sending ");
	Serial.print((10 + (4*(num_data_objects & 0x1F))), DEC);
	Serial.println(" bytes.");
	for (uint16_t i=0; i<(10 + (4*(num_data_objects & 0x1F))); i++) {
	Serial.print("0x");
	Serial.println(tx_buf[i], HEX);
	}
	Serial.println();
	*/
	temp = getReg(0x06);
	//sendBytes(tx_buf, (10+(4*(num_data_objects & 0x1F))) );
	//i2c.write_regs(fusb302_address, 0x43, tx_buf, (10+(4*(num_data_objects & 0x1F))) );
	setReg(0x06, (temp | (0x01))); // Flip on TX_START
}

int handle_rx(int16_t header, const uint32_t *payload)
{

	return 0;
}

int read_rx()
{
	uint8_t tmp[32] = {0};
	if (i2c.start() < 0) {
			return -1;
	}

	i2c.tx(fusb302_address&0xFE);

	if (i2c.wait_ack() < 0) {
			i2c.stop();
			return -1;
	}

	i2c.tx(0x43);
	i2c.wait_ack();

	i2c.start();
	i2c.tx((fusb302_address&0xFE)|0x01);
	i2c.wait_ack();
	
	do
	{
		tmp[0] = i2c.rx();
		i2c.send_ack();
	} while(tmp[0] & 0xE0 == 0xE0);

	tmp[0] = i2c.rx();
	i2c.send_ack();
	tmp[1] = i2c.rx();
	i2c.send_ack();
	
	int len = ( (tmp[1] >> 4) & 0x7 ) * 4;
	
	//PHex(len/4, xy(0, 24+8*got_int));
	PHex(tmp[0]&0xf, xy(0+3*6*got_int,40));
	PHex(len/4, xy(0+3*6*got_int,48));
	
	if (len > 28)
	{
		len = 28;
	}
	
	for(int i=0; i<len+4; i++)
	{
		tmp[i+2] = i2c.rx();
		if (i==len+4-1)
			i2c.send_nak();
		else
			i2c.send_ack();
	}

	i2c.stop();
	

	//handle_rx((tmp[1] << 8) | tmp[0], (uint32_t*)(tmp+2));
	int count = (tmp[1] >> 4) & 0x7;
	int type = tmp[0] & 0xf;
	
	if ((type == 1) && count > 0)
	{
		setReg(0x06, 0x40);	// TXFLUSH
		setReg(0x0b, 0x0f);	// power
		//uint8_t data[] = {0x12, 0x12, 0x12, 0x13, 0x86, 0x42, 0x1A, 0xc8, 0x20, 0x03, 0x23, 0xFF, 0x14, 0xA1};
		uint8_t data[] = {0x12, 0x12, 0x12, 0x13, 0x86, 0x42, 0x1A, 0x2C, 0xB1, 0x04, 0x23, 0xFF, 0x14, 0xA1};
		i2c.write_regs(fusb302_address, 0x43, data, sizeof(data));
		
		if (count > 5)
			count = 5;
		for(int i=0; i<count; i++)
		{
			//uint32_t v = *(uint32_t*)(tmp+2+4*i);
			uint8_t *p = tmp+2+4*i;
			uint32_t v = (p[3] << 24) | (p[2] << 16) | (p[1] << 8) | p[0];
			int voltage = ((v >> 10) & 0x3ff) * 50;
			int current = (v&0x3ff) * 10;
			
			PNum(voltage, xy(40, 8*i));
			PNum(current, xy(76, 8*i));
		}		
		
	}	
	
	return 0;
}

extern "C" void EXTI2_3_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line2);
	
	
	//PString("EXTI", 0);	
	
	while(!INT.read())
	{
		while (!(getReg(0x41) & 0x20))
		{
			if (read_rx() < 0)
				break;
			
			//uint8_t tmp[32];
			//i2c.read_regs(fusb302_address, 0x43, tmp, 32);
			
			got_int++;
		}
		PHex(getReg(0x41), 0);
		setReg(0x3e, 0);
		setReg(0x3f, 0);
		setReg(0x42, 0);		
	}
	PHex(getReg(0x41), 8);
}

int log_printf(const char*format, ...)
{
	char buffer[512];
	
	va_list args;
	va_start (args, format);
	int count = vsprintf (buffer,format, args);
	va_end (args);
	
	if (count < 0)
		return count;
	
	SEGGER_RTT_WriteString(0, buffer);
	
	return 0;
}

void init_exti()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_Init(&NVIC_InitStructure);
}

uint32_t v2;
int main()
{
	
	LEDH;//关闭背光
	PInit();//初始化显示屏
	LEDH;//关闭背光
	CB=BLA;//背景
	PSquare(0x00007F7F);
	PLogo();//显示开机画面
	LEDL;//打开背光
	systimer->delayms(1000);
	PSquare(0x00007F7F);//用背景黑色全屏清屏
	CF=WHI;//字体黑色
	i2c.set_speed(1);
	
	uint32_t vv[6] = {0x2801912c,0x2801912c};
	
	//handle_rx(0x5161, vv);
	
	// fusb302 version
	/*
	uint8_t reg01 = 0;
	i2c.read_reg(fusb302_address, 1, &reg01);
	//PString("VER:",0x0000);
	//PHex(reg01,0x1800);
	
	// 
	fusb302_init();
	
	// pd_reset()
	i2c.write_reg(fusb302_address, TCPC_REG_RESET, TCPC_REG_RESET_PD_RESET);
	
	// enable power for measure block
	i2c.write_reg(fusb302_address, 0x0b, 0x0F);
	
	//
	
	// measure CC1 & CC2
	uint8_t reg02 = 0;
	i2c.read_reg(fusb302_address, 2, &reg02);					// save reg02 for rolling back
	
	i2c.write_reg(fusb302_address, 2, (reg02 & 0xf3) | 0x04);	// measure CC1
	systimer->delayus(250);
	
	uint8_t cc1;
	i2c.read_reg(fusb302_address, 0x40, &cc1);
	cc1 &= 0x3;
	
	i2c.write_reg(fusb302_address, 2, (reg02 & 0xf3) | 0x08);	// measure CC2
	systimer->delayus(250);
	
	uint8_t cc2;
	i2c.read_reg(fusb302_address, 0x40, &cc2);
	cc2 &= 0x3;

	i2c.write_reg(fusb302_address, 2, reg02);	// roll back reg02
	
	
	if (cc1 > cc2)
		set_polarity(1);
	else
		set_polarity(0);		

	systimer->delayus(1000);
	*/
	
	/*
	setReg(0x0C, 0x03); // Reset FUSB302
	setReg(0x0B, 0x0F); // FULL POWER!
	setReg(0x07, 0x04); // Flush RX
	setReg(0x02, 0x0B); // Switch on MEAS_CC2
	//setReg(0x02, 0x07); // Switch on MEAS_CC1
	setReg(0x03, 0x26); // Enable BMC Tx on CC2
	//setReg(0x03, 0x25); // Enable BMC Tx on CC2
	//readAllRegs();
	*/
	
	// measure cc1 & cc2
	setReg(0x0c, 3);					// reset
	setReg(0x0b, 0x0f);					// power for measure
	uint8_t reg02 = getReg(0x02);
	setReg(0x02, (reg02 & 0xf3) | 0x04);
	systimer->delayus(250);
	uint8_t cc1 = getReg(0x40) & 0x3;
	setReg(0x02, (reg02 & 0xf3) | 0x08);
	systimer->delayus(250);
	uint8_t cc2 = getReg(0x40) & 0x3;
	setReg(0x02, reg02);	
	
	NVIC_DisableIRQ(EXTI2_3_IRQn);
	setReg(0x0c, 3);
	systimer->delayms(5);
	setReg(0x09, 0x07);
	setReg(0x0e, 0xff);
	setReg(0x0f, 0xfe);
	setReg(0x0a, 0xff);
	if (cc1>cc2)
	{
		setReg(0x02, 0x07);
		setReg(0x03, 0x25);
	}
	else
	{
		setReg(0x02, 0x0B);
		setReg(0x03, 0x26);
	}
	setReg(0x06, 0x00);
	setReg(0x0C, 0x02);
	setReg(0x0b, 0x0f);
	INT.set_mode(MODE_IN);
	init_exti();
	NVIC_EnableIRQ(EXTI2_3_IRQn);
		
	// 
	//sendPacket( 0, 2, 0, 1, 0, 0x7, NULL );
	//systimer->delayms(1);
	while(INT.read())
	{
		//NVIC_DisableIRQ(EXTI2_3_IRQn);
		//sprintf(tmp, "%02x,%d  ", getReg(0x41), INT.read() ? 1 : 0);
		//PString(tmp, 0);
		//NVIC_EnableIRQ(EXTI2_3_IRQn);
	}
	
	
	NVIC_DisableIRQ(EXTI2_3_IRQn);
	setReg(0x06, 0x40);	// TXFLUSH
	setReg(0x0b, 0x0f);	// power
	systimer->delayms(20);
	uint8_t data[] = {0x12, 0x12, 0x12, 0x13, 0x86, 0x42, 0x1A, 0xc8, 0x20, 0x03, 0x53, 0xFF, 0x14, 0xA1};
	//i2c.write_regs(fusb302_address, 0x43, data, sizeof(data));
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	
	
	//sprintf(tmp, "%02x ok       ", getReg(0x44));
	//PString(tmp, 0);

	// character size: 6*8 pixels
	/*
	i2c.write_reg(fusb302_address, 0x6, 40);
	i2c.write_reg(fusb302_address, 0xb, 0xf);
	*/
	uint8_t reg41;
	i2c.read_reg(fusb302_address, 0x44 , &reg41);
	
	systimer->delayms(200);
	
	/*
	PString("INT:", 0);
	PHex(got_int, xy(24, 0));
	PString("41: ", 8);
	PHex(getReg(0x41), xy(24, 8));
	PHex(INT.read(), 24);
	*/
	
	while(1)
	{
		log_printf("HelloRTT\n");
		systimer->delayms(500);
	}
	
	return 0;
}
