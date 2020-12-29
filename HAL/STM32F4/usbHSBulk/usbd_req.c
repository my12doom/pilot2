/**
  ******************************************************************************
  * @file    usbd_req.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the standard USB requests following chapter 9.
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
#include "usbd_req.h"
#include "usbd_ioreq.h"
#include "usbd_desc.h"

#ifdef	USE_PRINTF_DEBUG
#include <stdio.h>
#endif


/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_REQ
  * @brief USB standard requests module
  * @{
  */

/** @defgroup USBD_REQ_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Variables
  * @{
  */
extern __IO USB_OTG_DCTL_TypeDef SET_TEST_MODE;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint32_t USBD_ep_status __ALIGN_END  = 0; 				// USBD_ep_status = 0

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint32_t  USBD_default_cfg __ALIGN_END  = 0;				// USBD_default_cfg = 0

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint32_t  USBD_cfg_status __ALIGN_END  = 0;  				// USBD_cfg_status = 0

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ] __ALIGN_END ;			// 用于存放字符串描述符内容的buffer
/**
  * @}
  */


/** @defgroup USBD_REQ_Private_FunctionPrototypes
  * @{
  */
static void USBD_GetDescriptor(USB_OTG_CORE_HANDLE  *pdev,
                               USB_SETUP_REQ *req);

static void USBD_SetAddress(USB_OTG_CORE_HANDLE  *pdev,
                            USB_SETUP_REQ *req);

static void USBD_SetConfig(USB_OTG_CORE_HANDLE  *pdev,
                           USB_SETUP_REQ *req);

static void USBD_GetConfig(USB_OTG_CORE_HANDLE  *pdev,
                           USB_SETUP_REQ *req);

static void USBD_GetStatus(USB_OTG_CORE_HANDLE  *pdev,
                           USB_SETUP_REQ *req);

static void USBD_SetFeature(USB_OTG_CORE_HANDLE  *pdev,
                            USB_SETUP_REQ *req);

static void USBD_ClrFeature(USB_OTG_CORE_HANDLE  *pdev,
                            USB_SETUP_REQ *req);

static uint8_t USBD_GetLen(uint8_t *buf);
/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Functions
  * @{
  */

//========================================bmRequestType的D4~0表示请求的接收者，0-device,1-interface,2-endpoint,3-others,4~31reserved============================
/**
* @brief  USBD_StdDevReq
*         Handle standard usb device requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
/*	USBD_Status定义如下：
	USB_OK = 0;
	USB_BUSY = 1;
	USB_FAIL = 2;
*/
USBD_Status  USBD_StdDevReq (USB_OTG_CORE_HANDLE  *pdev, USB_SETUP_REQ  *req)			// 标准请求host to device
{
	USBD_Status ret = USBD_OK;

	switch (req->bRequest)
	{
	case USB_REQ_GET_DESCRIPTOR:

		USBD_GetDescriptor (pdev, req) ;
		break;

	case USB_REQ_SET_ADDRESS:
		USBD_SetAddress(pdev, req);
		break;

	case USB_REQ_SET_CONFIGURATION:
		USBD_SetConfig (pdev , req);
		break;

	case USB_REQ_GET_CONFIGURATION:
		USBD_GetConfig (pdev , req);
		break;

	case USB_REQ_GET_STATUS:
		USBD_GetStatus (pdev , req);
		break;


	case USB_REQ_SET_FEATURE:
		USBD_SetFeature (pdev , req);
		break;

	case USB_REQ_CLEAR_FEATURE:
		USBD_ClrFeature (pdev , req);
		break;

	default:
		USBD_CtlError(pdev , req);
		break;
	}

	return ret;
}

/**
* @brief  USBD_StdItfReq
*         Handle standard usb interface requests
* @param  pdev: USB OTG device instance
* @param  req: usb request
* @retval status
*/
USBD_Status  USBD_StdItfReq (USB_OTG_CORE_HANDLE  *pdev, USB_SETUP_REQ  *req)
{
	USBD_Status ret = USBD_OK;

	switch (pdev->dev.device_status)
	{
	case USB_OTG_CONFIGURED:			// USB_OTD_CONFIGURED = 3
#ifdef	USE_PRINTF_DEBUG
		printf("STD_ITF_REQ:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
		if (LOBYTE(req->wIndex) <= USBD_ITF_MAX_NUM)
		{
			pdev->dev.class_cb->Setup (pdev, req);

			if ((req->wLength == 0)&& (ret == USBD_OK))
			{
				USBD_CtlSendStatus(pdev);
			}
		}
		else
		{
			USBD_CtlError(pdev , req);
		}
		break;

	default:
		USBD_CtlError(pdev , req);
		break;
	}
	return ret;
}

/**
* @brief  USBD_StdEPReq
*         Handle standard usb endpoint requests
* @param  pdev: USB OTG device instance
* @param  req: usb request
* @retval status
*/
USBD_Status  USBD_StdEPReq (USB_OTG_CORE_HANDLE  *pdev, USB_SETUP_REQ  *req)
{

	uint8_t   ep_addr;
	USBD_Status ret = USBD_OK;

	ep_addr  = LOBYTE(req->wIndex);
#ifdef	USE_PRINTF_DEBUG
	printf("STD_EP_REQ:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
	printf("EP_ADDR = %X\r\n", ep_addr);
#endif
	switch (req->bRequest)
	{

	case USB_REQ_SET_FEATURE :

		switch (pdev->dev.device_status)
		{
		case USB_OTG_ADDRESSED:
			if ((ep_addr != 0x00) && (ep_addr != 0x80))
			{
				DCD_EP_Stall(pdev , ep_addr);
			}
			break;

		case USB_OTG_CONFIGURED:
			if (req->wValue == USB_FEATURE_EP_HALT)
			{
				if ((ep_addr != 0x00) && (ep_addr != 0x80))
				{
					DCD_EP_Stall(pdev , ep_addr);

				}
			}
			pdev->dev.class_cb->Setup (pdev, req);
			USBD_CtlSendStatus(pdev);

			break;

		default:
			USBD_CtlError(pdev , req);
			break;
		}
		break;

	case USB_REQ_CLEAR_FEATURE :

		switch (pdev->dev.device_status)
		{
		case USB_OTG_ADDRESSED:
			if ((ep_addr != 0x00) && (ep_addr != 0x80))
			{
				DCD_EP_Stall(pdev , ep_addr);
			}
			break;

		case USB_OTG_CONFIGURED:
			if (req->wValue == USB_FEATURE_EP_HALT)
			{
				if ((ep_addr != 0x00) && (ep_addr != 0x80))
				{
					DCD_EP_ClrStall(pdev , ep_addr);
					pdev->dev.class_cb->Setup (pdev, req);
				}
				USBD_CtlSendStatus(pdev);
			}
			break;

		default:
			USBD_CtlError(pdev , req);
			break;
		}
		break;

	case USB_REQ_GET_STATUS:
		switch (pdev->dev.device_status)
		{
		case USB_OTG_ADDRESSED:
			if ((ep_addr != 0x00) && (ep_addr != 0x80))
			{
				DCD_EP_Stall(pdev , ep_addr);
			}
			break;

		case USB_OTG_CONFIGURED:


			if ((ep_addr & 0x80)== 0x80)
			{
				if (pdev->dev.in_ep[ep_addr & 0x7F].is_stall)
				{
					USBD_ep_status = 0x0001;
				}
				else
				{
					USBD_ep_status = 0x0000;
				}
			}
			else if ((ep_addr & 0x80)== 0x00)
			{
				if (pdev->dev.out_ep[ep_addr].is_stall)
				{
					USBD_ep_status = 0x0001;
				}

				else
				{
					USBD_ep_status = 0x0000;
				}
			}
			USBD_CtlSendData (pdev,
			                  (uint8_t *)&USBD_ep_status,
			                  2);
			break;

		default:
			USBD_CtlError(pdev , req);
			break;
		}
		break;

	default:
		break;
	}
	return ret;
}
/**
* @brief  USBD_GetDescriptor
*         Handle Get Descriptor requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetDescriptor(USB_OTG_CORE_HANDLE  *pdev,
                               USB_SETUP_REQ *req)
{
	uint16_t len;
	uint8_t *pbuf;


	switch (req->wValue >> 8)			// GET Descriptor request的wValue的第一字节表示索引号，高字节表示描述符类型编号
	{
	case USB_DESC_TYPE_DEVICE:		// 1
#ifdef	USE_PRINTF_DEBUG
		printf("GetDescriptor->DeviceDescriptor:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
		pbuf = pdev->dev.usr_device->GetDeviceDescriptor(pdev->cfg.speed, &len);
		if ((req->wLength == 64) ||( pdev->dev.device_status == USB_OTG_DEFAULT))
		{
			len = 8;
		}
		break;

	case USB_DESC_TYPE_CONFIGURATION:
#ifdef	USE_PRINTF_DEBUG
		printf("GetDescriptor->ConfigDescriptor:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
		pbuf   = (uint8_t *)pdev->dev.class_cb->GetConfigDescriptor(pdev->cfg.speed, &len);
#ifdef USB_OTG_HS_CORE
		if ((pdev->cfg.speed == USB_OTG_SPEED_FULL )&&
		        (pdev->cfg.phy_itface  == USB_OTG_ULPI_PHY))
		{
			pbuf   = (uint8_t *)pdev->dev.class_cb->GetOtherConfigDescriptor(pdev->cfg.speed, &len);
		}
#endif
		pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
		pdev->dev.pConfig_descriptor = pbuf;
		break;

	case USB_DESC_TYPE_STRING:
		switch ((uint8_t)(req->wValue))
		{
		case USBD_IDX_LANGID_STR:
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->LanguageID:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			pbuf = pdev->dev.usr_device->GetLangIDStrDescriptor(pdev->cfg.speed, &len);
			break;

		case USBD_IDX_MFC_STR:
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->Manufactor:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			pbuf = pdev->dev.usr_device->GetManufacturerStrDescriptor(pdev->cfg.speed, &len);
			break;

		case USBD_IDX_PRODUCT_STR:
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->Product:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			pbuf = pdev->dev.usr_device->GetProductStrDescriptor(pdev->cfg.speed, &len);
			break;

		case USBD_IDX_SERIAL_STR:
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->SerialNumber:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			pbuf = pdev->dev.usr_device->GetSerialStrDescriptor(pdev->cfg.speed, &len);
			break;

		case USBD_IDX_CONFIG_STR:
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->ConfigString:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			pbuf = pdev->dev.usr_device->GetConfigurationStrDescriptor(pdev->cfg.speed, &len);
			break;

		case USBD_IDX_INTERFACE_STR:
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->InterfaceString:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			pbuf = pdev->dev.usr_device->GetInterfaceStrDescriptor(pdev->cfg.speed, &len);
			break;

		default:
#ifdef USB_SUPPORT_USER_STRING_DESC
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->UserString:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			pbuf = pdev->dev.class_cb->GetUsrStrDescriptor(pdev->cfg.speed, (req->wValue) , &len);
			break;
#else
#ifdef	USE_PRINTF_DEBUG
			printf("GetDescriptor->StringDescriptor->Unknown:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
			USBD_CtlError(pdev , req);
			return;
#endif /* USBD_CtlError(pdev , req); */
		}
		break;
	case USB_DESC_TYPE_DEVICE_QUALIFIER:
#ifdef	USE_PRINTF_DEBUG
		printf("GetDescriptor->DeviceQualifier:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
#ifdef USB_OTG_HS_CORE
		if (pdev->cfg.speed == USB_OTG_SPEED_HIGH  )
		{

			pbuf   = (uint8_t *)pdev->dev.class_cb->GetConfigDescriptor(pdev->cfg.speed, &len);

			USBD_DeviceQualifierDesc[4]= pbuf[14];
			USBD_DeviceQualifierDesc[5]= pbuf[15];
			USBD_DeviceQualifierDesc[6]= pbuf[16];

			pbuf = USBD_DeviceQualifierDesc;
			len  = USB_LEN_DEV_QUALIFIER_DESC;
			break;
		}
		else
		{
			USBD_CtlError(pdev , req);
			return;
		}
#else
		USBD_CtlError(pdev , req);
		return;
#endif

	case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
#ifdef	USE_PRINTF_DEBUG
		printf("GetDescriptor->OtherSpeedConfig:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
#ifdef USB_OTG_HS_CORE

		if (pdev->cfg.speed == USB_OTG_SPEED_HIGH  )
		{
			pbuf   = (uint8_t *)pdev->dev.class_cb->GetOtherConfigDescriptor(pdev->cfg.speed, &len);
			pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
			break;
		}
		else
		{
			USBD_CtlError(pdev , req);
			return;
		}
#else
		USBD_CtlError(pdev , req);
		return;
#endif


	default:
		USBD_CtlError(pdev , req);
		return;
	}

	if ((len != 0)&& (req->wLength != 0))
	{

		len = MIN(len , req->wLength);

		USBD_CtlSendData (pdev,
		                  pbuf,
		                  len);
#ifdef	USE_PRINTF_DEBUG
		printf("GetDescriptor,USB return %d bytes\r\n", len);
#endif
	}

}

/**
* @brief  USBD_SetAddress
*         Set device address
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetAddress(USB_OTG_CORE_HANDLE  *pdev,
                            USB_SETUP_REQ *req)
{
	uint8_t  dev_addr;
	// SetAddress request structure:bmRequestType=0,bRequest=0x05,wValue=deviceAddress,wIndex=0,wLength=0
#ifdef	USE_PRINTF_DEBUG
	printf("SetAddress:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
	if ((req->wIndex == 0) && (req->wLength == 0))
	{
		dev_addr = (uint8_t)(req->wValue) & 0x7F;		// 设备地址是1-127

		if (pdev->dev.device_status == USB_OTG_CONFIGURED)
		{
			USBD_CtlError(pdev , req);
		}
		else
		{
			pdev->dev.device_address = dev_addr;
			DCD_EP_SetAddress(pdev, dev_addr);
			USBD_CtlSendStatus(pdev);

			if (dev_addr != 0)
			{
				pdev->dev.device_status  = USB_OTG_ADDRESSED;
			}
			else
			{
				pdev->dev.device_status  = USB_OTG_DEFAULT;
			}
		}
	}
	else
	{
		USBD_CtlError(pdev , req);
	}
}

/**
* @brief  USBD_SetConfig
*         Handle Set device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetConfig(USB_OTG_CORE_HANDLE  *pdev,
                           USB_SETUP_REQ *req)
{

	static uint8_t  cfgidx;
	// SetConfig request structure:bmRequestType=0,bRequest=0x09,wValue=ConfigValue,wIndex=0,wLength=0
#ifdef	USE_PRINTF_DEBUG
	printf("SetConfig:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif

	cfgidx = (uint8_t)(req->wValue);

	if (cfgidx > USBD_CFG_MAX_NUM )
	{
		USBD_CtlError(pdev , req);
	}
	else
	{
		switch (pdev->dev.device_status)
		{
		case USB_OTG_ADDRESSED:
			if (cfgidx)
			{
				pdev->dev.device_config = cfgidx;
				pdev->dev.device_status = USB_OTG_CONFIGURED;
				USBD_SetCfg(pdev , cfgidx);
				USBD_CtlSendStatus(pdev);
			}
			else
			{
				USBD_CtlSendStatus(pdev);
			}
			break;

		case USB_OTG_CONFIGURED:
			if (cfgidx == 0)
			{
				pdev->dev.device_status = USB_OTG_ADDRESSED;
				pdev->dev.device_config = cfgidx;
				USBD_ClrCfg(pdev , cfgidx);
				USBD_CtlSendStatus(pdev);

			}
			else  if (cfgidx != pdev->dev.device_config)
			{
				/* Clear old configuration */
				USBD_ClrCfg(pdev , pdev->dev.device_config);

				/* set new configuration */
				pdev->dev.device_config = cfgidx;
				USBD_SetCfg(pdev , cfgidx);
				USBD_CtlSendStatus(pdev);
			}
			else
			{
				USBD_CtlSendStatus(pdev);
			}
			break;

		default:
			USBD_CtlError(pdev , req);
			break;
		}
	}
}

/**
* @brief  USBD_GetConfig
*         Handle Get device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetConfig(USB_OTG_CORE_HANDLE  *pdev,
                           USB_SETUP_REQ *req)
{
#ifdef	USE_PRINTF_DEBUG
	printf("GetConfig:bmRequest=%X bRequest=%X wValue=%X wIndex=%X wLength=%X\r\n", req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
#endif
	if (req->wLength != 1)
	{
		USBD_CtlError(pdev , req);
	}
	else
	{
		switch (pdev->dev.device_status )
		{
		case USB_OTG_ADDRESSED:

			USBD_CtlSendData (pdev,
			                  (uint8_t *)&USBD_default_cfg,			// 上面定义:USBD_default_cfg = 0
			                  1);
			break;

		case USB_OTG_CONFIGURED:

			USBD_CtlSendData (pdev,
			                  &pdev->dev.device_config,
			                  1);
			break;

		default:
			USBD_CtlError(pdev , req);
			break;
		}
	}
}

/**
* @brief  USBD_GetStatus
*         Handle Get Status request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetStatus(USB_OTG_CORE_HANDLE  *pdev,
                           USB_SETUP_REQ *req)
{


	switch (pdev->dev.device_status)
	{
	case USB_OTG_ADDRESSED:
	case USB_OTG_CONFIGURED:

#ifdef USBD_SELF_POWERED
		USBD_cfg_status = USB_CONFIG_SELF_POWERED;
#else
		USBD_cfg_status = 0x00;
#endif

		if (pdev->dev.DevRemoteWakeup)
		{
			USBD_cfg_status |= USB_CONFIG_REMOTE_WAKEUP;
		}

		USBD_CtlSendData (pdev,
		                  (uint8_t *)&USBD_cfg_status,
		                  2);
		break;

	default :
		USBD_CtlError(pdev , req);
		break;
	}
}


/**
* @brief  USBD_SetFeature
*         Handle Set device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetFeature(USB_OTG_CORE_HANDLE  *pdev,
                            USB_SETUP_REQ *req)
{

	USB_OTG_DCTL_TypeDef     dctl;
	uint8_t test_mode = 0;

	if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
	{
		pdev->dev.DevRemoteWakeup = 1;
		pdev->dev.class_cb->Setup (pdev, req);
		USBD_CtlSendStatus(pdev);
	}

	else if ((req->wValue == USB_FEATURE_TEST_MODE) &&
	         ((req->wIndex & 0xFF) == 0))
	{
		dctl.d32 = USB_OTG_READ_REG32(&pdev->regs.DREGS->DCTL);

		test_mode = req->wIndex >> 8;
		switch (test_mode)
		{
		case 1: // TEST_J
			dctl.b.tstctl = 1;
			break;

		case 2: // TEST_K
			dctl.b.tstctl = 2;
			break;

		case 3: // TEST_SE0_NAK
			dctl.b.tstctl = 3;
			break;

		case 4: // TEST_PACKET
			dctl.b.tstctl = 4;
			break;

		case 5: // TEST_FORCE_ENABLE
			dctl.b.tstctl = 5;
			break;
		}
		SET_TEST_MODE = dctl;
		pdev->dev.test_mode = 1;
		USBD_CtlSendStatus(pdev);
	}

}


/**
* @brief  USBD_ClrFeature
*         Handle clear device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_ClrFeature(USB_OTG_CORE_HANDLE  *pdev,
                            USB_SETUP_REQ *req)
{
	switch (pdev->dev.device_status)
	{
	case USB_OTG_ADDRESSED:
	case USB_OTG_CONFIGURED:
		if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
		{
			pdev->dev.DevRemoteWakeup = 0;
			pdev->dev.class_cb->Setup (pdev, req);
			USBD_CtlSendStatus(pdev);
		}
		break;

	default :
		USBD_CtlError(pdev , req);
		break;
	}
}

/**
* @brief  USBD_ParseSetupRequest
*         Copy buffer into setup structure			// 解释建立请求的函数
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_ParseSetupRequest( USB_OTG_CORE_HANDLE  *pdev,
                             USB_SETUP_REQ *req)
{	// 把USB接收到的请求复制到USB_SETUP_REQ的结构体中
	req->bmRequest     = *(uint8_t *)  (pdev->dev.setup_packet);
	req->bRequest      = *(uint8_t *)  (pdev->dev.setup_packet +  1);
	req->wValue        = SWAPBYTE      (pdev->dev.setup_packet +  2);
	req->wIndex        = SWAPBYTE      (pdev->dev.setup_packet +  4);
	req->wLength       = SWAPBYTE      (pdev->dev.setup_packet +  6);

	pdev->dev.in_ep[0].ctl_data_len = req->wLength  ;
	pdev->dev.device_state = USB_OTG_EP0_SETUP;
}

/**
* @brief  USBD_CtlError
*         Handle USB low level Error
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_CtlError( USB_OTG_CORE_HANDLE  *pdev,
                    USB_SETUP_REQ *req)
{
#ifdef	USE_PRINTF_DEBUG
	printf("USBD_CtlError!\r\n");
#endif
	DCD_EP_Stall(pdev , 0x80);
	DCD_EP_Stall(pdev , 0);
	USB_OTG_EP0_OutStart(pdev);
}


/**
  * @brief  USBD_GetString
  *         Convert Ascii string into unicode one
  * @param  desc : descriptor buffer
  * @param  unicode : Formatted string buffer (unicode)
  * @param  len : descriptor length
  * @retval None
  */
void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)		// 把字符串转化为UNICODE编码
{
	uint8_t idx = 0;

	if (desc != NULL)
	{
		*len =  USBD_GetLen(desc) * 2 + 2;
		unicode[idx++] = *len;
		unicode[idx++] =  USB_DESC_TYPE_STRING;

		while (*desc != NULL)
		{
			unicode[idx++] = *desc++;
			unicode[idx++] =  0x00;
		}
	}
}

/**
  * @brief  USBD_GetLen
  *         return the string length
   * @param  buf : pointer to the ascii string buffer
  * @retval string length
  */
static uint8_t USBD_GetLen(uint8_t *buf)
{
	uint8_t  len = 0;

	while (*buf != NULL)
	{
		len++;
		buf++;
	}

	return len;
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
