/**
  ******************************************************************************
  * @file    usbd_core.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides all the USBD core functions.
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
#include "usbd_core.h"
#include "usbd_req.h"
#include "usbd_ioreq.h"
#include "usb_dcd_int.h"
#include "usb_bsp.h"
#include "../F4HSBulk.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/


/** @defgroup USBD_CORE
* @brief usbd core module
* @{
*/

/** @defgroup USBD_CORE_Private_TypesDefinitions
* @{
*/
/**
* @}
*/


/** @defgroup USBD_CORE_Private_Defines
* @{
*/

/**
* @}
*/


/** @defgroup USBD_CORE_Private_Macros
* @{
*/
/**
* @}
*/




/** @defgroup USBD_CORE_Private_FunctionPrototypes
* @{
*/
static uint8_t USBD_SetupStage(USB_OTG_CORE_HANDLE *pdev);
static uint8_t USBD_DataOutStage(USB_OTG_CORE_HANDLE *pdev , uint8_t epnum);
static uint8_t USBD_DataInStage(USB_OTG_CORE_HANDLE *pdev , uint8_t epnum);
static uint8_t USBD_SOF(USB_OTG_CORE_HANDLE  *pdev);
static uint8_t USBD_Reset(USB_OTG_CORE_HANDLE  *pdev);
static uint8_t USBD_Suspend(USB_OTG_CORE_HANDLE  *pdev);
static uint8_t USBD_Resume(USB_OTG_CORE_HANDLE  *pdev);
#ifdef VBUS_SENSING_ENABLED
static uint8_t USBD_DevConnected(USB_OTG_CORE_HANDLE  *pdev);
static uint8_t USBD_DevDisconnected(USB_OTG_CORE_HANDLE  *pdev);
#endif
static uint8_t USBD_IsoINIncomplete(USB_OTG_CORE_HANDLE  *pdev);
static uint8_t USBD_IsoOUTIncomplete(USB_OTG_CORE_HANDLE  *pdev);
static uint8_t  USBD_RunTestMode (USB_OTG_CORE_HANDLE  *pdev) ;
/**
* @}
*/

/** @defgroup USBD_CORE_Private_Variables
* @{
*/

__IO USB_OTG_DCTL_TypeDef SET_TEST_MODE;

USBD_DCD_INT_cb_TypeDef USBD_DCD_INT_cb =
{
	USBD_DataOutStage,
	USBD_DataInStage,
	USBD_SetupStage,
	USBD_SOF,
	USBD_Reset,
	USBD_Suspend,
	USBD_Resume,
	USBD_IsoINIncomplete,
	USBD_IsoOUTIncomplete,
#ifdef VBUS_SENSING_ENABLED
	USBD_DevConnected,
	USBD_DevDisconnected,
#endif
};

USBD_DCD_INT_cb_TypeDef  *USBD_DCD_INT_fops = &USBD_DCD_INT_cb;
/**
* @}
*/

/** @defgroup USBD_CORE_Private_Functions
* @{
*/

/**
* @brief  USBD_Init			// 初始化设备库，加载类驱动和用户回调函数
*         Initailizes the device stack and load the class driver
* @param  pdev: device instance
* @param  core_address: USB OTG core ID
* @param  class_cb: Class callback structure address
* @param  usr_cb: User callback structure address
* @retval None
*/
void USBD_Init(USB_OTG_CORE_HANDLE *pdev,			// 实例句柄
               USB_OTG_CORE_ID_TypeDef coreID,		// FS或者HS的ID, HS=0
               USBD_DEVICE *pDevice,                // &USR_desc  定义于usbd_desc.c，用于加载各种描述符
               USBD_Class_cb_TypeDef *class_cb, 	// &USBD_CDC_cb 定义于usbd_cdc_core.c  CDC类回调函数
               USBD_Usr_cb_TypeDef *usr_cb)			// &USR_cb 定义于usbd_usr.c，用户回调函数
{
	/* Hardware Init */
	USB_OTG_BSP_Init(pdev);  					// 定义于usb_bsp.c 用于初始化IO口

	USBD_DeInit(pdev);						// 恢复默认的USB设置

	/*Register class and user callbacks */
	pdev->dev.class_cb = class_cb;
	pdev->dev.usr_cb = usr_cb;
	pdev->dev.usr_device = pDevice;

	/* set USB OTG core params */
	DCD_Init(pdev , coreID);					// 定义于usb_dcd.c 初始化寄存器地址，端点，内核，设备模式等

	/* Upon Init call usr callback */
	pdev->dev.usr_cb->Init();					// 调用usbd_usr.c的init函数初始化用户代码

	/* Enable Interrupts */
	USB_OTG_BSP_EnableInterrupt(pdev);		// 调用usb_bsp.c的中断NVIC配置来使能中断
}

/**
* @brief  USBD_DeInit
*         Re-Initialize th device library
* @param  pdev: device instance
* @retval status: status
*/
USBD_Status USBD_DeInit(USB_OTG_CORE_HANDLE *pdev)
{
	/* Software Init */

	return USBD_OK;
}


uint8_t ctl_rx_buf[64];
USB_SETUP_REQ ctl_out_req;
/**
* @brief  USBD_SetupStage
*         Handle the setup stage
* @param  pdev: device instance
* @retval status
*/
static uint8_t USBD_SetupStage(USB_OTG_CORE_HANDLE *pdev)
{
	USB_SETUP_REQ req;

	USBD_ParseSetupRequest(pdev , &req);

	/* bmRequestType
	D7:数据传输方向
		0:host->device
		1:device->host
	D6~5:请求的类型
		0:standard
		1:class
		2:vendor
		3:reserved
	D4~0:请求的接受者
		0:device
		1:interface
		2:endpoint
		3:others
		4~31:reserved
	*/

	// TODO: vendor!

	printf("**bmRequest %02x**, length %d\n", req.bmRequest, req.wLength);

	if ((req.bmRequest&0x7f) == 0x40)		// Vendor request
	{
		if (req.bmRequest&0x80)
		{
			// "IN" request
			/*
			if (req.wLength)
			USBD_CtlSendData (pdev, ctl_rx_buf, req.wLength);
			else
			USBD_CtlReceiveStatus(pdev);
			*/

			printf("vendor IN request, %d bytes max\n", req.wLength);

			control_verndor_transfer t  =
			{
				1, req.bmRequest&0x7f, req.bRequest, req.wValue, req.wIndex, req.wLength
			};

			F4cb(control_IN, &t, sizeof(t));
		}
		else
		{
			printf("vendor OUT request, %d bytes\n", req.wLength);

			ctl_out_req = req;
			if (req.wLength)
				USBD_CtlPrepareRx(pdev, ctl_rx_buf, req.wLength);
			else
				USBD_CtlSendStatus(pdev);
		}
		return USBD_OK;
	}

	switch (req.bmRequest & 0x1F) 	// 只判断D4~0
	{
	case USB_REQ_RECIPIENT_DEVICE:
		USBD_StdDevReq (pdev, &req);		// 这几个函数都在usbd_req.c中
		break;

	case USB_REQ_RECIPIENT_INTERFACE:
		USBD_StdItfReq(pdev, &req);
		break;

	case USB_REQ_RECIPIENT_ENDPOINT:
		USBD_StdEPReq(pdev, &req);
		break;

	default:
		DCD_EP_Stall(pdev , req.bmRequest & 0x80);		// usb_dcd.c中
		break;
	}
	return USBD_OK;
}

/**
* @brief  USBD_DataOutStage
*         Handle data out stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t USBD_DataOutStage(USB_OTG_CORE_HANDLE *pdev , uint8_t epnum)
{
	USB_OTG_EP *ep;

	if (epnum == 0) 		// 如果是端点0
	{
		ep = &pdev->dev.out_ep[0];
		if ( pdev->dev.device_state == USB_OTG_EP0_DATA_OUT)
		{
			if (ep->rem_data_len > ep->maxpacket)		// 如果端点待发送的数据长度大于端点最大长度，那么分两次以上发送，并改变ep结构体相关的值
			{
				ep->rem_data_len -=  ep->maxpacket;

				if (pdev->cfg.dma_enable == 1)
				{
					/* in slave mode this, is handled by the RxSTSQLvl ISR */
					ep->xfer_buff += ep->maxpacket;
				}
				USBD_CtlContinueRx (pdev,
				                    ep->xfer_buff,
				                    MIN(ep->rem_data_len ,ep->maxpacket));
			}
			else
			{	// 调用usbd_class_core.c中的xxx_EP0_RxReady(xxx)函数
				if ((pdev->dev.class_cb->EP0_RxReady != NULL)&&
				        (pdev->dev.device_status == USB_OTG_CONFIGURED))
				{
					pdev->dev.class_cb->EP0_RxReady(pdev); 		// 调用来做什么?

					int data_count = USBD_GetRxCount(pdev, 0);

					printf("CTRL_RX:%d bytes, req=%d, index=%d, value=%d\n", data_count, ctl_out_req.bRequest, ctl_out_req.wIndex, ctl_out_req.wValue);
					control_verndor_transfer t =
					{
						0, ctl_out_req.bmRequest & 0x7f, ctl_out_req.bRequest, ctl_out_req.wValue, ctl_out_req.wIndex,
						data_count, ctl_rx_buf
					};

					F4cb(control_OUT, &t, sizeof(t));
				}
				USBD_CtlSendStatus(pdev);
			}
		}
	}	// 如果不是端点0，那么直接调用类回调函数的DataOut函数
	else if ((pdev->dev.class_cb->DataOut != NULL)&&
	         (pdev->dev.device_status == USB_OTG_CONFIGURED))
	{
		pdev->dev.class_cb->DataOut(pdev, epnum);
	}
	return USBD_OK;
}

/**
* @brief  USBD_DataInStage
*         Handle data in stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
static uint8_t USBD_DataInStage(USB_OTG_CORE_HANDLE *pdev , uint8_t epnum)
{
	USB_OTG_EP *ep;

	if (epnum == 0)
	{
		ep = &pdev->dev.in_ep[0];
		if ( pdev->dev.device_state == USB_OTG_EP0_DATA_IN)
		{
			if (ep->rem_data_len > ep->maxpacket)
			{
				ep->rem_data_len -=  ep->maxpacket;
				if (pdev->cfg.dma_enable == 1)
				{
					/* in slave mode this, is handled by the TxFifoEmpty ISR */
					ep->xfer_buff += ep->maxpacket;
				}
				USBD_CtlContinueSendData (pdev,
				                          ep->xfer_buff,
				                          ep->rem_data_len);
			}
			else
			{ /* last packet is MPS multiple, so send ZLP packet */
				if ((ep->total_data_len % ep->maxpacket == 0) &&
				        (ep->total_data_len >= ep->maxpacket) &&
				        (ep->total_data_len < ep->ctl_data_len ))
				{

					USBD_CtlContinueSendData(pdev , NULL, 0);
					ep->ctl_data_len = 0;
				}
				else
				{
					if ((pdev->dev.class_cb->EP0_TxSent != NULL)&&
					        (pdev->dev.device_status == USB_OTG_CONFIGURED))
					{
						pdev->dev.class_cb->EP0_TxSent(pdev);
					}
					USBD_CtlReceiveStatus(pdev);
				}
			}
		}
		if (pdev->dev.test_mode == 1)
		{
			USBD_RunTestMode(pdev);
			pdev->dev.test_mode = 0;
		}
	}
	else if ((pdev->dev.class_cb->DataIn != NULL)&&
	         (pdev->dev.device_status == USB_OTG_CONFIGURED))
	{
		pdev->dev.class_cb->DataIn(pdev, epnum);
	}
	return USBD_OK;
}




/**
* @brief  USBD_RunTestMode
*         Launch test mode process
* @param  pdev: device instance
* @retval status
*/
static uint8_t  USBD_RunTestMode (USB_OTG_CORE_HANDLE  *pdev)
{
	USB_OTG_WRITE_REG32(&pdev->regs.DREGS->DCTL, SET_TEST_MODE.d32);
	return USBD_OK;
}

/**
* @brief  USBD_Reset
*         Handle Reset event
* @param  pdev: device instance
* @retval status
*/

static uint8_t USBD_Reset(USB_OTG_CORE_HANDLE  *pdev)
{
	/* Open EP0 OUT */
	DCD_EP_Open(pdev,
	            0x00,
	            USB_OTG_MAX_EP0_SIZE,
	            EP_TYPE_CTRL);

	/* Open EP0 IN */
	DCD_EP_Open(pdev,
	            0x80,
	            USB_OTG_MAX_EP0_SIZE,
	            EP_TYPE_CTRL);

	/* Upon Reset call usr call back */
	pdev->dev.device_status = USB_OTG_DEFAULT;
	pdev->dev.usr_cb->DeviceReset(pdev->cfg.speed);

	return USBD_OK;
}

/**
* @brief  USBD_Resume
*         Handle Resume event
* @param  pdev: device instance
* @retval status
*/

static uint8_t USBD_Resume(USB_OTG_CORE_HANDLE  *pdev)
{
	/* Upon Resume call usr call back */
	pdev->dev.usr_cb->DeviceResumed();
	pdev->dev.device_status = pdev->dev.device_old_status;
	pdev->dev.device_status = USB_OTG_CONFIGURED;
	return USBD_OK;
}


/**
* @brief  USBD_Suspend
*         Handle Suspend event
* @param  pdev: device instance
* @retval status
*/

static uint8_t USBD_Suspend(USB_OTG_CORE_HANDLE  *pdev)
{
	pdev->dev.device_old_status = pdev->dev.device_status;
	pdev->dev.device_status  = USB_OTG_SUSPENDED;
	/* Upon Resume call usr call back */
	pdev->dev.usr_cb->DeviceSuspended();
	return USBD_OK;
}


/**
* @brief  USBD_SOF
*         Handle SOF event
* @param  pdev: device instance
* @retval status
*/

static uint8_t USBD_SOF(USB_OTG_CORE_HANDLE  *pdev)
{
	if (pdev->dev.class_cb->SOF)
	{
		pdev->dev.class_cb->SOF(pdev);
	}
	return USBD_OK;
}
/**
* @brief  USBD_SetCfg
*        Configure device and start the interface
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status
*/

USBD_Status USBD_SetCfg(USB_OTG_CORE_HANDLE  *pdev, uint8_t cfgidx)
{
	pdev->dev.class_cb->Init(pdev, cfgidx);

	/* Upon set config call usr call back */
	pdev->dev.usr_cb->DeviceConfigured();
	return USBD_OK;
}

/**
* @brief  USBD_ClrCfg
*         Clear current configuration
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status: USBD_Status
*/
USBD_Status USBD_ClrCfg(USB_OTG_CORE_HANDLE  *pdev, uint8_t cfgidx)
{
	pdev->dev.class_cb->DeInit(pdev, cfgidx);
	return USBD_OK;
}

/**
* @brief  USBD_IsoINIncomplete
*         Handle iso in incomplete event
* @param  pdev: device instance
* @retval status
*/
static uint8_t USBD_IsoINIncomplete(USB_OTG_CORE_HANDLE  *pdev)
{
	pdev->dev.class_cb->IsoINIncomplete(pdev);
	return USBD_OK;
}

/**
* @brief  USBD_IsoOUTIncomplete
*         Handle iso out incomplete event
* @param  pdev: device instance
* @retval status
*/
static uint8_t USBD_IsoOUTIncomplete(USB_OTG_CORE_HANDLE  *pdev)
{
	pdev->dev.class_cb->IsoOUTIncomplete(pdev);
	return USBD_OK;
}

#ifdef VBUS_SENSING_ENABLED
/**
* @brief  USBD_DevConnected
*         Handle device connection event
* @param  pdev: device instance
* @retval status
*/
static uint8_t USBD_DevConnected(USB_OTG_CORE_HANDLE  *pdev)
{
	pdev->dev.usr_cb->DeviceConnected();
	pdev->dev.connection_status = 1;
	return USBD_OK;
}

/**
* @brief  USBD_DevDisconnected
*         Handle device disconnection event
* @param  pdev: device instance
* @retval status
*/
static uint8_t USBD_DevDisconnected(USB_OTG_CORE_HANDLE  *pdev)
{
	pdev->dev.usr_cb->DeviceDisconnected();
	pdev->dev.class_cb->DeInit(pdev, 0);
	pdev->dev.connection_status = 0;
	return USBD_OK;
}
#endif
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

