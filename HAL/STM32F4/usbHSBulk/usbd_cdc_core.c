/**
  ******************************************************************************
  * @file    usbd_cdc_core.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class

  *           @note
  *             For the Abstract Control Model, this core allows only transmitting the requests to
  *             lower layer dispatcher (ie. usbd_cdc_vcp.c/.h) which should manage each request and
  *             perform relative actions.
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
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
#include "usbd_cdc_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"

#include <stm32F4xx_gpio.h>
#include <stm32F4xx_flash.h>

#include "../F4HSBulk.h"
int F4cb(int event, void *p, int size);

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_cdc 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup usbd_cdc_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   CDC Device library callbacks
 *********************************************/
static uint8_t  usbd_cdc_Init        (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cdc_DeInit      (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cdc_Setup       (void  *pdev, USB_SETUP_REQ *req);
static uint8_t  usbd_cdc_EP0_RxReady  (void *pdev);
static uint8_t  usbd_cdc_DataIn      (void *pdev, uint8_t epnum);
static uint8_t  usbd_cdc_DataOut     (void *pdev, uint8_t epnum);
static uint8_t  usbd_cdc_SOF         (void *pdev);

/*********************************************
   CDC specific management functions
 *********************************************/
//static void Handle_USBAsynchXfer (void *pdev);
static uint8_t  *USBD_cdc_GetCfgDesc (uint8_t speed, uint16_t *length);
#ifdef USE_USB_OTG_HS  
static uint8_t  *USBD_cdc_GetOtherCfgDesc (uint8_t speed, uint16_t *length);
#endif
/**
  * @}
  */ 

/** @defgroup usbd_cdc_Private_Variables
  * @{
  */ 
extern CDC_IF_Prop_TypeDef  APP_FOPS;
extern uint8_t USBD_DeviceDesc   [USB_SIZ_DEVICE_DESC];			// USB_SIZ_DEVICE_DESC = 18 设备描述符18个字节

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc  [USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_OtherCfgDesc  [USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN static __IO uint32_t  usbd_cdc_AltSet  __ALIGN_END = 0;		// usbd_cdc_AltSet?

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [CDC_DATA_MAX_PACKET_SIZE_OUT] __ALIGN_END ;			// USB_Rx_Buffer, CDC_DATA_MAX_PACKET_SIZE = 512

// #ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
//   #if defined ( __ICCARM__ ) /*!< IAR Compiler */
//     #pragma data_alignment=4   
//   #endif
// #endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
// __ALIGN_BEGIN uint8_t APP_Rx_Buffer   [APP_RX_DATA_SIZE] __ALIGN_END ; 					


// #ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
//   #if defined ( __ICCARM__ ) /*!< IAR Compiler */
//     #pragma data_alignment=4   
//   #endif
// #endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
// __ALIGN_BEGIN uint8_t CmdBuff[CDC_CMD_PACKET_SZE] __ALIGN_END ;							// Control Endpoint Packet size 8

//
// uint8_t test_buf[512] = "Why data transfer error by bulk endpoint?";
uint8_t usb_tx_state  = 0;
uint8_t buf1_is_using = 0;
uint8_t buf2_is_using = 0;
int init_done = 0;

/* CDC interface class callbacks structure */
USBD_Class_cb_TypeDef  USBD_CDC_cb = 							// CDC 类回调函数结构体
{
  usbd_cdc_Init,
  usbd_cdc_DeInit,
  usbd_cdc_Setup,
  NULL,                 /* EP0_TxSent, */
  usbd_cdc_EP0_RxReady,
  usbd_cdc_DataIn,
  usbd_cdc_DataOut,
  usbd_cdc_SOF,
  NULL,
  NULL,     
  USBD_cdc_GetCfgDesc,
#ifdef USE_USB_OTG_HS   
  USBD_cdc_GetOtherCfgDesc, /* use same config as per FS */
#endif /* USE_USB_OTG_HS  */
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc[USB_CDC_CONFIG_DESC_SIZ]  __ALIGN_END =				// USB配置描述符集合
{
  /*Configuration Descriptor*/																// 配置描述符
  0x09,   /* bLength: Configuration Descriptor size */										// 配置描述符共有9个字节
  USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */				// 配置描述符类型:0x02
  USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */			// 配置描述符集合总长度:配置描述符(9)+接口描述符(9)+端点IN描述符(7)+端点OUT描述符(7) = 32字节
  0x00,									  /* wTotalLength:no of returned bytes */			// 
  0x01,   /* bNumInterfaces: 1 interface */													// 这个配置有多少个接口,这里为1
  0x01,   /* bConfigurationValue: Configuration value */									// 这个配置的配置号,USB设备可能支持多个配置
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */		// 描述该配置的描述符索引号,为0表示没有字符串
  0xC0,   /* bmAttributes: self powered */													// D7保留必为1,D6为1为自供电，D5为0不支持远程唤醒,D4~D0保留为0
  0x32,   /* MaxPower 100 mA */																// 表示该设备需要从总线获取的电流值大小
	
  /*Data class interface descriptor*/														// 接口描述符
  0x09,   /* bLength: Endpoint Descriptor size */											// 9个字节
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */									// 接口描述符类型:0x04(字符串描述符为0x03)
  0x00,   /* bInterfaceNumber: Number of Interface */										// 第1个接口的编号,从0开始.这里只有一个接口
  0x00,   /* bAlternateSetting: Alternate setting */										// 该接口的备用编号,很少用到,设置为0.
  0x02,   /* bNumEndpoints: Two endpoints used */											// 使用两个端点
  0xFF,   /* bInterfaceClass:  */															// 接口使用的类,子类,协议,通常在接口中定义设备的功能,而在设备描述符中将这3个值设置为0即可
  0xFF,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */																	// 描述该接口的字符串描述符的索引号,為0表示沒有
  
  /*Endpoint OUT Descriptor*/																// OUT端点描述符:7个字节
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */						// 端点描述符类型为:0x05
  CDC_OUT_EP,                        /* bEndpointAddress */									// EP1 out:0x01(bit8=0表示OUT EP)
  0x02,                              /* bmAttributes: Bulk */								// 端点属性:最低2位为0控制传输,1等时传输,2bulk传输,3中断传输(如果是等时传输类型,其他bit会有其他功能)
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE_OUT),  /* wMaxPacketSize: */									// 端点支持的最大包长度
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE_OUT),															// 改动端点每个帧包含的事务数量能否提升速度？Pls refer to the usb2.0 spec:page271	
//  ((HIBYTE(CDC_DATA_MAX_PACKET_SIZE)) | 0x10),		// (3 per microframe)
  0x01,                              /* bInterval: ignore for Bulk transfer */				// 该端点的查询的时间, Pls refer to the usb2.0 spec:page271	
  
  /*Endpoint IN Descriptor*/																// IN端点描述符:7个字节
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */									// EP1 IN:// EP1 out:0x81(bit8=1表示IN EP)
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE_IN),  /* wMaxPacketSize: */    								// 512bytes
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE_IN),
//  ((HIBYTE(CDC_DATA_MAX_PACKET_SIZE)) | 0x10),		// (3 per microframe)
  0x01                               /* bInterval: ignore for Bulk transfer */
} ;

#ifdef USE_USB_OTG_HS
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */ 
__ALIGN_BEGIN uint8_t usbd_cdc_OtherCfgDesc[USB_CDC_CONFIG_DESC_SIZ]  __ALIGN_END =			// USB2.0协议新增:Other Config Descriptor集合
{ 
  // Other speed config descriptor	
  0x09,              /* 0  bLength */
  0x07,      		 /* 1  bDescriptorType */
  0x12,              /* 2  wTotalLength */
  0x00,              /* 3  wTotalLength */
  0x01,              /* 4  bNumInterface: Number of interfaces*/
  0x00,              /* 5  bConfigurationValue */
  0x00,              /* 6  iConfiguration */
  0x80,              /* 7  bmAttributes */
  0xC8,              /* 8  bMaxPower */
  
  // 接口描述符 
  0x09,              /* 0 bLength */
  0x04,              /* 1 bDescriptorType */
  0x00,              /* 2 bInterfaceNumber */
  0x00,              /* 3 bAlternateSetting */
  0x00,              /* 4 bNumEndpoints */
  0x00,              /* 5 bInterfaceClass */
  0x00,              /* 6 bInterfaceSubclass */
  0x00,              /* 7 bInterfaceProtocol */
  0x00,              /* 8 iInterface */
};
#endif /* USE_USB_OTG_HS  */

/**
  * @}
  */ 

/** @defgroup usbd_cdc_Private_Functions
  * @{
  */ 

/**
  * @brief  usbd_cdc_Init
  *         Initilaize the CDC interface				// 初始化CDC接口
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cdc_Init (void  *pdev, 
                               uint8_t cfgidx)
{
  uint8_t *pbuf;

  /* Open EP IN */		// 打开EP1端点，设置EP1包大小，传输方式
  DCD_EP_Open(pdev,
              CDC_IN_EP,						// 0x81
              CDC_DATA_MAX_PACKET_SIZE_IN,			// 512 bytes
              USB_OTG_EP_BULK);					// bulk
  
  /* Open EP OUT */
  DCD_EP_Open(pdev,
              CDC_OUT_EP,						// 0x01
              CDC_DATA_MAX_PACKET_SIZE_OUT,			// 512 bytes
              USB_OTG_EP_BULK);					// bulk
// EP2 屏蔽  
//   /* Open Command IN EP */
//   DCD_EP_Open(pdev,
//               CDC_CMD_EP,
//               CDC_CMD_PACKET_SZE,
//               USB_OTG_EP_INT);
  
	// 指针指向设备描述符，usbd_desc.c中定义(此处我改动了头文件，把它改为了自定义设备)
  pbuf = (uint8_t *)USBD_DeviceDesc;
  pbuf[4] = DEVICE_CLASS_CDC;			// 修改设备描述符中的第5和第6字节，即设备类，设备子类为CDC的编号，编号定义位于usbd_cdc_core.h
  pbuf[5] = DEVICE_SUBCLASS_CDC;
  
  /* Initialize the Interface physical components */
  // APP_FOPS在usbd_conf.h中的定义：#define APP_FOPS       VCP_fops
  // 在usbd_cdc_vcp.c中有定义：
  /*	CDC_IF_Prop_TypeDef VCP_fops = 
		{
		  VCP_Init,
		  VCP_DeInit,
		  VCP_Ctrl,
		  VCP_DataTx,
		  VCP_DataRx
		};
		// 在usbd_cdc_core.h中有定义：
		typedef struct _CDC_IF_PROP
		{
		  uint16_t (*pIf_Init)     (void);   
		  uint16_t (*pIf_DeInit)   (void);   
		  uint16_t (*pIf_Ctrl)     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
		  uint16_t (*pIf_DataTx)   (uint8_t* Buf, uint32_t Len);
		  uint16_t (*pIf_DataRx)   (uint8_t* Buf, uint32_t Len);
		}
		CDC_IF_Prop_TypeDef;
	*/
    
//  APP_FOPS.pIf_Init();			// 所以这里其实是执行了usbd_cdc_vcp.c中的VCP_Init函数,这个函数是初始化了STM32的物理串口，但是我只需要用usb收发数据，所以注释掉了。

  /* Prepare Out endpoint to receive next packet */		// EP1 OUT准备接受PC传过来的数据
  /*
  DCD_EP_PrepareRx(pdev,
                   CDC_OUT_EP,
                   (uint8_t*)(USB_Rx_Buffer),		// buffer在上面的定义为512字节
                   sizeof(USB_Rx_Buffer));		// 在usb_cdc_core.h中：#define CDC_DATA_OUT_PACKET_SIZE               CDC_DATA_MAX_PACKET_SIZE				即512
  */
  
  F4cb(rx_ready, NULL, 0);
  
  init_done = 1;
  
  return USBD_OK;
}

/**
  * @brief  usbd_cdc_Init
  *         DeInitialize the CDC layer				// 关闭端点
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cdc_DeInit (void  *pdev, 
                                 uint8_t cfgidx)
{
  /* Open EP IN */
  DCD_EP_Close(pdev,
              CDC_IN_EP);
  
  /* Open EP OUT */
  DCD_EP_Close(pdev,
              CDC_OUT_EP);
  
//   /* Open Command IN EP */
//   DCD_EP_Close(pdev,
//               CDC_CMD_EP);

  /* Restore default state of the Interface physical components */
//  APP_FOPS.pIf_DeInit();			// 在usbd_cdc_vcp.c中，只简单的返回了USB_OK
  
  return USBD_OK;
}

/**
  * @brief  usbd_cdc_Setup
  *         Handle the CDC specific requests			// 处理CDC类特殊的请求
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  usbd_cdc_Setup (void  *pdev, 
                                USB_SETUP_REQ *req)			// USB_SETUP_REQ为8字节的请求
{
  uint16_t len=USB_CDC_DESC_SIZ;		// 在usbd_cdc_core.h中:#define USB_CDC_DESC_SIZ                       (67-9)
  uint8_t  *pbuf=usbd_cdc_CfgDesc + 9;	// 指向配置描述符的第一个接口描述符的第一个字节，为什么？
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)		// USB_REQ_TYPE_MASK在usbd_def.h中定义为0x60(0110 0000)
  {
    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
	
    /* Standard Requests -------------------------------*/
	case USB_REQ_TYPE_STANDARD:			// 0x00		// 标准请求中的第6,5bit都是0
    switch (req->bRequest)		// 标准请求在usbd_def.h中有定义具体数值
    {
    case USB_REQ_GET_DESCRIPTOR: 				// 6
      if( (req->wValue >> 8) == CDC_DESCRIPTOR_TYPE)	// 在描述符请求中，wValue表示的是描述符类型和索引，第一字节表示索引号，第二字节表示描述符的类型编号
      {		// CDC_DESCRIPTOR_TYPE因为USB底层函数不识别，所以返回给用户回调函数处理，这在st的usb库说明文档有说明，这里改成纯bulk设备后，应该不需要了。
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
        pbuf = usbd_cdc_Desc; 
#else
        pbuf = usbd_cdc_CfgDesc + 9 + (9 * USBD_ITF_MAX_NUM);		// USBD_ITF_MAX_NUM = 1, 也就是指向了/*Header Functional Descriptor*/的开头
#endif 
		//   
        len = MIN(USB_CDC_DESC_SIZ , req->wLength);					// USB_CDC_DESC_SIZ = 67-9:减去9个字节的配置描述符，剩下的是接口描述符开始以后的各种描述符
      }
      // USB控制传输(EP0控制传输)
      USBD_CtlSendData (pdev, pbuf, len);
      break;
	  
    // 获取接口  ,在这个请求中，host请求的数据量为1字节,这个请求只在USB状态机的配置状态下有效
    case USB_REQ_GET_INTERFACE :		//0x0A
      USBD_CtlSendData (pdev,
                        (uint8_t *)&usbd_cdc_AltSet, 1);		// usbd_cdc_AltSet的值为0，这里只返回一个0，为什么?
      break;
    // 设置接口  This request allows the host to select an alternate setting for the specified interface.
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
      {
        usbd_cdc_AltSet = (uint8_t)(req->wValue);		// usbd_cdc_AltSet是usbd cdc alternate setting的意思
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
      }
      break;
    }
  }
  return USBD_OK;
}

/**
  * @brief  usbd_cdc_EP0_RxReady
  *         Data received on control endpoint			// 表示EP0已经收到了数据
  * @param  pdev: device device instance
  * @retval status
  */
static uint8_t  usbd_cdc_EP0_RxReady (void  *pdev)			// 这个函数可以清空了。
{ 
//   if (cdcCmd != NO_CMD)			// usbd_cdc_core.h定义为0xFF
//   {
//     /* Process the data */
//     APP_FOPS.pIf_Ctrl(cdcCmd, CmdBuff, cdcLen);			// 根据cdcCmd命令处理数据
//     
//     /* Reset the command variable to default value */
//     cdcCmd = NO_CMD;
//   }
  
  return USBD_OK;
}

/**
  * @brief  usbd_audio_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cdc_DataIn (void *pdev, uint8_t epnum)				// 当usb发送完成后被调用
{
	if(usb_tx_state == 1)
	{
		usb_tx_state = 0;
//		extern int data_valid;
//		extern int data_usb;
		F4cb(tx_done, NULL, 0);
		
//		data_valid &= ~(data_usb+1);			
	}
	
	return USBD_OK;
}
/**
  * @brief  usbd_cdc_DataOut
  *         Data received on non-control Out endpoint		// USB接收PC发送过来的数据
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */

static uint8_t  usbd_cdc_DataOut (void *pdev, uint8_t epnum)
{  
	// USB data will be immediately processed, this allow next USB traffic being NAKed till the end of the application Xfer */	
	F4cb(rx_done, NULL, ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count);
	F4cb(rx_ready, NULL, 0);

	return USBD_OK;
}


/**
  * @brief  Start Of Frame event management
  * @param  pdev: instance
  * @retval None
  */
static uint8_t usbd_cdc_SOF (void *pdev)
{
	if(usb_tx_state == 0 && init_done == 1)
	{
		if (F4cb(tx_ready, NULL, 0) == 0)
			usb_tx_state = 1;
	}
	
	return USBD_OK;
}

/**
  * @brief  USBD_cdc_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speedDCD_EP_Tx (pdev, CDC_IN_EP, ((uint8_t*)&adc_data[0]) + (data_usb?32768:0)  , 32768);
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_cdc_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_cdc_CfgDesc);
  return usbd_cdc_CfgDesc;
}

/**
  * @brief  USBD_cdc_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
#ifdef USE_USB_OTG_HS 
static uint8_t  *USBD_cdc_GetOtherCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_cdc_OtherCfgDesc);
  return usbd_cdc_OtherCfgDesc;
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
