#ifndef _DCMI_2640_H
#define _DCMI_2640_H

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#ifdef __cplusplus
extern "C"
{
#endif
//#define USE_WAVESHARE
#define DK_HAL_STM32F407_V_1_0 
#define USE_INTER_RAM
#ifdef USE_INTER_RAM
	#define jpeg_buf_size 160*120*2/4+4
#else
	#define jpeg_buf_size 1
#endif
#define DCMI_DR_ADDRESS       0x50050028

#define DCMI_TIMEOUT_MAX               10000
#define OV2640_DEVICE_WRITE_ADDRESS    0x60
#define OV2640_DEVICE_READ_ADDRESS     0x61

extern uint32_t dcmi_image_buffer_32bit_1[jpeg_buf_size];
	
typedef enum   
{
  OV9655_CAMERA            =   0x00,	 /* Use OV9655 Camera */
  OV2640_CAMERA            =   0x01      /* Use OV2640 Camera */
}Camera_TypeDef;

/* Image Sizes enumeration */
typedef enum   
{
  BMP_QQVGA             =   0x00,	    /* BMP Image QQVGA 160x120 Size */
  BMP_QVGA              =   0x01,           /* BMP Image QVGA 320x240 Size */
  JPEG_160x120          =   0x02,	    /* JPEG Image 160x120 Size */
  JPEG_176x144          =   0x03,	    /* JPEG Image 176x144 Size */
  JPEG_320x240          =   0x04,	    /* JPEG Image 320x240 Size */
  JPEG_352x288          =   0x05,	    /* JPEG Image 352x288 Size */
  JPEG_640x480          =   0x06
}camera_format;

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t Manufacturer_ID1;
	uint8_t Manufacturer_ID2;
	uint8_t PIDH;
	uint8_t PIDL;
}OV2640_IDTypeDef;

void dcmi_hw_init(void);
void dcmi_dma_init(uint32_t image_buffer_address,uint16_t buffer_size,camera_format ImageFormat);
void dcmi_dma_init_7725(uint32_t image_buffer_address,uint16_t buffer_size);
void dcmi_dma_enable(void);
void dcmi_dma_disable(void);
void dcmi_it_init();
void dma_it_init();
void dcmi_start(void);
void dcmi_stop(void);
void dma_swap_buffers(void);
void dcmi_clock_init();
int jpeg_data_process();

uint8_t OV2640_WriteReg(uint16_t Addr, uint8_t Data);
uint8_t OV2640_ReadReg(uint16_t Addr);

#ifdef __cplusplus
}
#endif
#endif