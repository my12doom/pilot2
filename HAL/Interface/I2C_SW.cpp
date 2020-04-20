#include <stdlib.h>
#include "II2C.h"

namespace HAL
{	
I2C_SW::I2C_SW()
:m_speed_tick(5)
,m_SDA(NULL)
,m_SCL(NULL)
{
}

I2C_SW::I2C_SW(IGPIO *SCL, IGPIO *SDA)
:m_speed_tick(5)
{
	init(SCL, SDA);
}

I2C_SW::~I2C_SW()
{
}

int I2C_SW::init(IGPIO *SCL, IGPIO *SDA)
{
	m_SDA = SDA;
	m_SCL = SCL;
	m_SDA->set_mode(MODE_OUT_OpenDrain);
	m_SCL->set_mode(MODE_OUT_OpenDrain);
	
	I2C_Stop();
	
	return 0;
}

int I2C_SW::set_speed(int speed)		// speed in hz
{
	// TODO: calculate speed tick
	m_speed_tick = speed;
	
	return 0;
}

int I2C_SW::read_regs(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count)
{
	int i;

	if (!I2C_Start()) {
			return -1;
	}

	I2C_SendByte(SlaveAddress&0xFE);

	if (!I2C_WaitAck()) {
			I2C_Stop();
			return -1;
	}

	I2C_SendByte(startRegister);
	I2C_WaitAck();

	I2C_Start();
	I2C_SendByte((SlaveAddress&0xFE)|0x01);
	I2C_WaitAck();

	for(i=0; i<count; i++)
	{
		out[i] = I2C_ReceiveByte();
		if (i==count-1)
			I2C_SendNoAck();
		else
			I2C_SendAck();
	}

	I2C_Stop();
	
	return 0;
}

int I2C_SW::write_regs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count)
{
	int i;
    if (!I2C_Start()) {
        return -1;
    }

    I2C_SendByte(SlaveAddress&0xFE);

    if (!I2C_WaitAck()) {
        I2C_Stop();
        return -1;
    }

    I2C_SendByte(startRegister&0xFF);
    I2C_WaitAck();

    for(i=0; i<count; i++)
    {
        I2C_SendByte(data[i]&0xFF);
        
        if (!I2C_WaitAck())
        {
            I2C_Stop();
            return -1;
        }
    }

    I2C_Stop();

    return 0;
}


#define SCL_HI     m_SCL->write(true)
#define SCL_LO     m_SCL->write(false)
#define SDA_HI     m_SDA->write(true)
#define SDA_LO     m_SDA->write(false)

bool I2C_SW::SDA_STATE()
{
	bool v;
	//m_SDA->set_mode(MODE_IN);
	v = m_SDA->read();
	//m_SDA->set_mode(MODE_OUT_OpenDrain);
	
	return v;
}
bool I2C_SW::SCL_STATE()
{
	bool v;
	//m_SDA->set_mode(MODE_IN);
	v = m_SCL->read();
	//m_SDA->set_mode(MODE_OUT_OpenDrain);

	return v;
}

void I2C_SW::I2C_Delay(void)
{
    volatile int speedTick = m_speed_tick;
    while (speedTick) {
       speedTick--;
    }
}

uint8_t I2C_SW::I2C_Start(void)
{
    SDA_HI;
    I2C_SCLHigh();
    I2C_Delay();
    if (!SDA_STATE()) {
        //DB_Print("I2C_Start:BUSY!\n");
        return 0;
    }
    SDA_LO;
    I2C_Delay();
    if (SDA_STATE()) {
        //DB_Print("I2C_Start:BUS ERROR!\n");
        return 0;
    }
    SDA_LO;
    I2C_Delay();
    return 1;
}

void I2C_SW::I2C_Stop(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_LO;
    I2C_Delay();
    I2C_SCLHigh();
    I2C_Delay();
    SDA_HI;
    I2C_Delay();
}

void I2C_SW::I2C_SendAck(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_LO;
    I2C_Delay();
    I2C_SCLHigh();
    I2C_Delay();
    SCL_LO;
    I2C_Delay();
}

void I2C_SW::I2C_SendNoAck(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_HI;
    I2C_Delay();
    I2C_SCLHigh();
    I2C_Delay();
    SCL_LO;
    I2C_Delay();
}

uint8_t I2C_SW::I2C_WaitAck(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_HI;
    I2C_Delay();
    I2C_SCLHigh();
    I2C_Delay();
    if (SDA_STATE()) {
      SCL_LO;
      return 0;
    }
    SCL_LO;
    return 1;
}

uint8_t I2C_SW::I2C_SCLHigh(void)
{
	volatile int retry = 999;
	SCL_HI;
	while(retry && !SCL_STATE())
		retry --;

	return retry > 0;
}

void I2C_SW::I2C_SendByte(uint8_t Data)
{
    uint8_t i = 8;
    while (i--) {
        SCL_LO;
        I2C_Delay();
        if (Data&0x80) {
            SDA_HI;
        }
        else {
            SDA_LO;
        }
        Data <<= 1;
        I2C_Delay();
        I2C_SCLHigh();
        I2C_Delay();
    }
    SCL_LO;
}

uint8_t I2C_SW::I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t Data = 0;
    SDA_HI;
    while (i--) {
        Data <<= 1;
        SCL_LO;
        I2C_Delay();
        I2C_SCLHigh();
        I2C_Delay();
        if (SDA_STATE()) {
            Data |= 0x01;
        }
    }
    SCL_LO;
    return Data;
}

// low level control
int I2C_SW::start()		// return -1 if bus busy or any error.
{
	SDA_HI;
	I2C_SCLHigh();
	I2C_Delay();
	if (!SDA_STATE())
		return -1;
	SDA_LO;
	I2C_Delay();
	if (SDA_STATE())
		return -1;
	SDA_LO;
	I2C_Delay();
	return 0;
}

int I2C_SW::stop()			// return -1 on any error.
{
	SCL_LO;
	I2C_Delay();
	SDA_LO;
	I2C_Delay();
	I2C_SCLHigh();
	I2C_Delay();
	SDA_HI;
	I2C_Delay();

	return 0;
}

int I2C_SW::send_ack()		// return -1 on any error.
{
	SCL_LO;
	I2C_Delay();
	SDA_LO;
	I2C_Delay();
	I2C_SCLHigh();
	I2C_Delay();
	SCL_LO;
	I2C_Delay();

	return 0;
}
int I2C_SW::send_nak()		// return -1 on any error.
{
	SCL_LO;
	I2C_Delay();
	SDA_HI;
	I2C_Delay();
	I2C_SCLHigh();
	I2C_Delay();
	SCL_LO;
	I2C_Delay();

	return 0;
}

int I2C_SW::wait_ack()		// return 0 on success, -1 on any error, 1 if a nak received.
{
	SCL_LO;
	I2C_Delay();
	SDA_HI;
	I2C_Delay();
	I2C_SCLHigh();
	I2C_Delay();
	if (SDA_STATE()) {
		SCL_LO;
		return -1;
	}
	SCL_LO;
	return 0;
}

int I2C_SW::tx(uint8_t tx)	// send a byte.
{
	I2C_SendByte(tx);
	return 0;
}

uint8_t I2C_SW::rx()	// send a byte.
{
	return I2C_ReceiveByte();
}


}
