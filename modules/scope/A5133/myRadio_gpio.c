#include "myRadio_gpio.h"
#include "misc.h"

RADIO_GPIO_CALLBACK gpioCallback;
static irqCallback_ts myIrqCallback_extiLine2;
uint8_t sdioMode = 255;

void gpio_delayUs(uint32_t time)
{
    uint32_t i, j;
    i = time;
    while (i --)
    {
        for ( j = 0; j < 1; j++)
        {
            ;
        }
    }
}
//---------------------------射频SPI驱动部分---------------------
void BOARD_A5133_SCS_H(void)
{
    GPIO_WriteBit(RF_A5133_SCS, BOARD_PIN_H);
}
void BOARD_A5133_SCS_L(void)
{
    GPIO_WriteBit(RF_A5133_SCS, BOARD_PIN_L);
}
void BOARD_A5133_SCK_H(void)
{
    GPIO_WriteBit(BOARD_GPIO_SPI_CLK, BOARD_PIN_H);
}
void BOARD_A5133_SCK_L(void)
{
    GPIO_WriteBit(BOARD_GPIO_SPI_CLK, BOARD_PIN_L);
}
void SET_A5133_SDIO_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    if (sdioMode == 0)
    {
        return;
    }
    sdioMode = 0;
    GPIO_InitStructure.GPIO_Pin = RF_A5133_SDIO_PIN;           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_SDIO_PORT, &GPIO_InitStructure);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
void SET_A5133_SDIO_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    if (sdioMode == 1)
    {
        return;
    }
    sdioMode = 1;
    GPIO_InitStructure.GPIO_Pin = RF_A5133_SDIO_PIN;           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_SDIO_PORT, &GPIO_InitStructure);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
void BOARD_A5133_SDIO_H(void)
{
    SET_A5133_SDIO_OUT();
    GPIO_WriteBit(RF_A5133_SDIO, BOARD_PIN_H);
}
void BOARD_A5133_SDIO_L(void)
{
    SET_A5133_SDIO_OUT();
    GPIO_WriteBit(RF_A5133_SDIO, BOARD_PIN_L);
}

uint8_t READ_RF_A5133_SDIO(void)
{
    SET_A5133_SDIO_IN();
    return GPIO_ReadInputDataBit(RF_A5133_SDIO);
}
//---------------------------射频驱动IO部分---------------------
void SET_A5133_GPIO1_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RF_A5133_GPIO1_PIN;           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_GPIO1_PORT, &GPIO_InitStructure);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
void SET_A5133_GPIO1_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RF_A5133_GPIO1_PIN;           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_GPIO1_PORT, &GPIO_InitStructure);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
void RF_A5133_GPIO1_H(void)
{
    GPIO_WriteBit(RF_A5133_GPIO1, BOARD_PIN_H);
}
void RF_A5133_GPIO1_L(void)
{
    GPIO_WriteBit(RF_A5133_GPIO1, BOARD_PIN_L);
}

void SET_A5133_GPIO2_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RF_A5133_GPIO2_PIN;           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_GPIO2_PORT, &GPIO_InitStructure);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
void SET_A5133_GPIO2_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RF_A5133_GPIO2_PIN;           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_GPIO2_PORT, &GPIO_InitStructure);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
void RF_A5133_GPIO2_H(void)
{
    GPIO_WriteBit(RF_A5133_GPIO2, BOARD_PIN_H);
}
void RF_A5133_GPIO2_L(void)
{
    GPIO_WriteBit(RF_A5133_GPIO2, BOARD_PIN_L);
}
void RF_A5133_CKO_H(void)
{
    GPIO_WriteBit(RF_A5133_CKO, BOARD_PIN_H);
}
void RF_A5133_CKO_L(void)
{
    GPIO_WriteBit(RF_A5133_CKO, BOARD_PIN_L);
}
void RF_EXT_PA_RE_H(void)
{
    GPIO_WriteBit(RF_EXTPA_RE, BOARD_PIN_H);
}
void RF_EXT_PA_RE_L(void)
{
    GPIO_WriteBit(RF_EXTPA_RE, BOARD_PIN_L);
}
void RF_EXT_PA_TE_H(void)
{
    GPIO_WriteBit(RF_EXTPA_TE, BOARD_PIN_H);
}
void RF_EXT_PA_TE_L(void)
{
    GPIO_WriteBit(RF_EXTPA_TE, BOARD_PIN_L);
}
uint8_t READ_RF_A5133_GPIO1(void)
{
    return GPIO_ReadInputDataBit(RF_A5133_GPIO1);
}
/**
 * @brief 引脚控制
 * 
 * @param sta =true，设置进入低功耗的引脚状态
 *            =false，设置退出低功耗的引脚状态
 */
void myRadio_gpio_intoLwPwr(bool sta)
{
    if (sta == false)
    {

    }
    else
    {

    }
}
void extiLine2_callback(uint8_t status, uint32_t param)
{
    gpioCallback(1);
}
void myRadio_gpio_irq_init()
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    myIrqCallback_extiLine2.thisCb = extiLine2_callback;
    EXTILINE2_callbackRegiste(&myIrqCallback_extiLine2); 

    GPIO_InitStructure.GPIO_Pin = RF_A5133_GPIO1_PIN;         
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
    GPIO_Init(RF_A5133_GPIO1_PORT, &GPIO_InitStructure);

    EXTI_ClearITPendingBit(EXTI_Line2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
    /* Enable and set EXTI1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void myRadio_gpio_init(RADIO_GPIO_CALLBACK cb)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RF_A5133_SDIO_PIN;           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_SDIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = RF_A5133_SCK_PIN;            
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_SCK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RF_A5133_SCS_PIN;            
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_SCS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RF_A5133_GPIO1_PIN;            
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_GPIO1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RF_A5133_GPIO2_PIN;            
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_GPIO2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RF_A5133_CKO_PIN;            
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(RF_A5133_CKO_PORT, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = RF_EXTPA_RE_PIN;            
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    // GPIO_Init(RF_EXTPA_RE_PORT, &GPIO_InitStructure);
    // GPIO_InitStructure.GPIO_Pin = RF_EXTPA_TE_PIN;            
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    // GPIO_Init(RF_EXTPA_TE_PORT, &GPIO_InitStructure);

    BOARD_A5133_SCS_H();
    BOARD_A5133_SCK_L();
    BOARD_A5133_SDIO_H();
    RF_A5133_GPIO1_H();
    myRadio_gpio_irq_init();
	if(cb)
    gpioCallback = cb;
}
/**
 * @brief 
 *      下降沿后，RF芯片会改变SDIO的状态
 *      上升沿后才能读取SDIO状态
 * @return uint8_t 
 */
uint8_t myRadioSpi_rByte(void)
{
    uint8_t i, temp;
    temp = 0;   

    // BOARD_A5133_SCK_L();
    for(i = 0; i < 8; i ++)
    {
        gpio_delayUs(10);
        temp <<= 1;
        if(READ_RF_A5133_SDIO())
        {
          temp ++; 
        }
        BOARD_A5133_SCK_H();
        gpio_delayUs(10);
        BOARD_A5133_SCK_L();
    }
	return temp;
}
/**
 * @brief 
 *      在上升沿之前需设置好SDIO的状态
 * @param byteToWrite 
 */
void myRadioSpi_wByte(uint8_t byteToWrite)
{
    uint8_t i;

    BOARD_A5133_SCK_L();
    for(i = 0; i < 8; i ++)
    {
        if(byteToWrite & 0x80)
        {
          BOARD_A5133_SDIO_H();
        }
        else
        {
          BOARD_A5133_SDIO_L();
        } 

        byteToWrite <<= 1;
        gpio_delayUs(10);
        BOARD_A5133_SCK_H();
        gpio_delayUs(10);
        BOARD_A5133_SCK_L();
    }
}
uint8_t myRadioSpi_rwByte(uint8_t byteToWrite)
{
    uint8_t i, temp;
    temp = 0;   

    // BOARD_A5133_SCK_L();
    for(i = 0; i < 8; i ++)
    {
        if(byteToWrite & 0x80)
        {
          BOARD_A5133_SDIO_H();
        }
        else
        {
          BOARD_A5133_SDIO_L();
        } 

        byteToWrite <<= 1;
        gpio_delayUs(10);
        BOARD_A5133_SCK_H();
        temp <<= 1;
        if(READ_RF_A5133_SDIO())
        {
          temp ++; 
        }
        BOARD_A5133_SCK_L();
    }
	return temp;
}

void myRadioSpi_wBuffer(uint8_t* pData, uint8_t len)
{
  uint8_t i;
  
  for(i = 0; i < len; i++)
  {
    myRadioSpi_rwByte(*pData);
    pData ++;
  }
}

void myRadioSpi_rBuffer(uint8_t* pData, uint8_t len)
{
  uint8_t i;
  
  for(i = 0; i < len; i++)
  {
    *pData = myRadioSpi_rwByte(0xFF);
    pData ++;
  }
}
void myRadioSpi_rwBuffer(uint8_t* pDataR, uint8_t* pDataW, uint8_t len)
{
  uint8_t i;
  
  for(i = 0; i < len; i++)
  {
    *pDataR = myRadioSpi_rwByte(*pDataW);
    pDataR ++;
    pDataW ++;
  }
}
