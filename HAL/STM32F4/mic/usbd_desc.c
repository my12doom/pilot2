/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the USBD descriptors and string formating method.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#include "usbd_conf.h"
#include "usb_regs.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_DESC 
  * @brief USBD descriptors module
  * @{
  */ 

/** @defgroup USBD_DESC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Defines
  * @{
  */ 


#define USBD_VID                        0x0483

#ifdef STM32F2XX
 #define USBD_PID                       0x5730
#else
 #define USBD_PID                       0x5730
#endif /* STM32F2XX */

/** @defgroup USB_String_Descriptors
  * @{
  */ 
//#define USBD_LANGID_STRING              0x409		// ENGLISH
#define USBD_LANGID_STRING              0x0804
#define USBD_MANUFACTURER_STRING        "xjtu"

#define USBD_PRODUCT_FS_STRING          "nage waimai"
#define USBD_PRODUCT_HS_STRING          "nage waimai"
#define USBD_SERIALNUMBER_FS_STRING     "00000000034E"
#define USBD_SERIALNUMBER_HS_STRING     "00000000034F"

#define USBD_CONFIGURATION_FS_STRING    "AUDIO Config"
#define USBD_INTERFACE_FS_STRING        "AUDIO Interface"

#define USBD_CONFIGURATION_HS_STRING    "AUDIO Config"
#define USBD_INTERFACE_HS_STRING        "AUDIO Interface"

/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Variables
  * @{
  */ 

USBD_DEVICE USR_desc =
{
  USBD_USR_DeviceDescriptor,
  USBD_USR_LangIDStrDescriptor, 
  USBD_USR_ManufacturerStrDescriptor,
  USBD_USR_ProductStrDescriptor,
  USBD_USR_SerialStrDescriptor,
  USBD_USR_ConfigStrDescriptor,
  USBD_USR_InterfaceStrDescriptor,
  
};


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
 #if defined   (__CC_ARM) /*!< ARM Compiler */
  __align(4)
 #elif defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
 #elif defined (__GNUC__) /*!< GNU Compiler */
 #pragma pack(4)
 #elif defined  (__TASKING__) /*!< TASKING Compiler */
  __align(4)
 #endif /* __CC_ARM */
#endif


/* USB Standard Device Descriptor */
uint8_t USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_OTG_MAX_EP0_SIZE,      /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID),           /*idVendor*/
    HIBYTE(USBD_PID),           /*idVendor*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_CFG_MAX_NUM            /*bNumConfigurations*/
  }
  ; /* USB_DeviceDescriptor */


/* USB Standard Device Descriptor */
uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/* USB Standard Device Descriptor */
uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID] =
{
     USB_SIZ_STRING_LANGID,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING), 
};
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Functions
  * @{
  */ 

/**
* @brief  USBD_USR_DeviceDescriptor 
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length)
{
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

/**
* @brief  USBD_USR_LangIDStrDescriptor 
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length)
{
  *length =  sizeof(USBD_LangIDDesc);  
  return USBD_LangIDDesc;
}


/**
* @brief  USBD_USR_ProductStrDescriptor 
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ProductStrDescriptor( uint8_t speed , uint16_t *length)
{
 
  
    USBD_GetString (USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
	static const uint8_t desc[] = {8,USB_DESC_TYPE_STRING, 0xC1, 0x94, 0x1D, 0x4e, 0xA6, 0x9e};
		*length = 8;
		memcpy(USBD_StrDesc, desc, *length);
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ManufacturerStrDescriptor 
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ManufacturerStrDescriptor( uint8_t speed , uint16_t *length)
{
  USBD_GetString (USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_SerialStrDescriptor 
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed  == USB_OTG_SPEED_HIGH)
  {    
    USBD_GetString (USBD_SERIALNUMBER_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_USR_ConfigStrDescriptor 
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed  == USB_OTG_SPEED_HIGH)
  {  
    USBD_GetString (USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length); 
  }
  return USBD_StrDesc;  
}


/**
* @brief  USBD_USR_InterfaceStrDescriptor 
*         return the interface string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString (USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;  
}

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

