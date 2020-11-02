#include "usbd_audio_core.h"

/*********************************************
   AUDIO Device library callbacks
 *********************************************/
static uint8_t  usbd_audio_Init       (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_audio_DeInit     (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_audio_Setup      (void  *pdev, USB_SETUP_REQ *req);
static uint8_t  usbd_audio_EP0_RxReady(void *pdev);
static uint8_t  usbd_audio_DataIn     (void *pdev, uint8_t epnum);
static uint8_t  usbd_audio_DataOut    (void *pdev, uint8_t epnum);
static uint8_t  usbd_audio_SOF        (void *pdev);
static uint8_t  usbd_audio_OUT_Incplt (void  *pdev);
static uint8_t  usbd_audio_IN_Incplt (void  *pdev);


/*********************************************
   AUDIO Requests management functions
 *********************************************/
static void AUDIO_Req_GetCurrent(void *pdev, USB_SETUP_REQ *req);
static void AUDIO_Req_SetCurrent(void *pdev, USB_SETUP_REQ *req);
static uint8_t  *USBD_audio_GetCfgDesc (uint8_t speed, uint16_t *length);
/**
  * @}
  */ 

/** @defgroup usbd_audio_Private_Variables
  * @{
  */ 

extern volatile int16_t volume;// = 0x3f;
extern volatile int volume_changed;// = 0;
extern volatile uint8_t muted;// = 0;

/* Main Buffer for Audio Control Rrequests transfers and its relative variables */
uint8_t  AudioCtlBuf[64];
uint8_t  AudioCtlCmd = 0;
uint32_t AudioCtlLen = 0;
uint8_t  AudioCtlUnit = 0;

static uint32_t PlayFlag = 0;

static __IO uint32_t  usbd_audio_AltSet = 0;
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE];
extern uint8_t buffer_ready;

/* AUDIO interface class callbacks structure */
USBD_Class_cb_TypeDef  AUDIO_cb = 
{
  usbd_audio_Init,
  usbd_audio_DeInit,
  usbd_audio_Setup,
  NULL, /* EP0_TxSent */
  usbd_audio_EP0_RxReady,
  usbd_audio_DataIn,
  usbd_audio_DataOut,
  usbd_audio_SOF,
  usbd_audio_IN_Incplt,
  usbd_audio_OUT_Incplt,
  USBD_audio_GetCfgDesc,
#ifdef USB_OTG_HS_CORE  
  USBD_audio_GetCfgDesc, /* use same config as per FS */
#endif    
};

/* USB AUDIO device Configuration Descriptor */
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE] =
{
    /* USB Microphone Configuration Descriptor */
    0x09,//sizeof(USB_CFG_DSC),    // Size of this descriptor in bytes
    USB_CONFIGURATION_DESCRIPTOR_TYPE,                // CONFIGURATION descriptor type (0x02)
    LOBYTE(AUDIO_CONFIG_DESC_SIZE),       /* wTotalLength  109 bytes*/
    HIBYTE(AUDIO_CONFIG_DESC_SIZE),
    2,                      // Number of interfaces in this cfg
    1,                      // Index value of this configuration
    0,                      // Configuration string index
    0x80,       // Attributes, see usb_device.h
    50,                     // Max power consumption (2X mA)

    /* USB Microphone Standard AC Interface Descriptor  */
    0x09,//sizeof(USB_INTF_DSC),   // Size of this descriptor in bytes
    USB_INTERFACE_DESCRIPTOR_TYPE, // INTERFACE descriptor type
    0x00,    // Interface Number
    0x00,                          // Alternate Setting Number
    0x00,                          // Number of endpoints in this intf
    USB_DEVICE_CLASS_AUDIO,        // Class code
    AUDIO_SUBCLASS_AUDIOCONTROL,   // Subclass code
    0x00,                          // Protocol code
    0x00,                          // Interface string index


    /* USB Microphone Class-specific AC Interface Descriptor  (CODE == 9)*/
    0x09,                         // Size of this descriptor, in bytes.
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, // CS_INTERFACE Descriptor Type 0x24
    AUDIO_CONTROL_HEADER,         // HEADER descriptor subtype 0x01
    0x00,0x01,                    // Audio Device compliant to the USB Audio specification version 1.00
    0x1E +9+7+9,0x00,                    // Total number of bytes returned for the class-specific AudioControl interface descriptor.
                                  // Includes the combined length of this descriptor header and all Unit and Terminal descriptors.
    0x01,                         // The number of AudioStreaming interfaces in the Audio Interface Collection to which this AudioControl interface belongs
    0x01,                         // AudioStreaming interface 1 belongs to this AudioControl interface.


    // USB Microphone Input Terminal Descriptor
    0x0C,                         // Size of the descriptor, in bytes
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, // CS_INTERFACE Descriptor Type
    AUDIO_CONTROL_INPUT_TERMINAL,    // INPUT_TERMINAL descriptor subtype
    0x01,                         // ID of this Terminal.
    0x07,0x10,                    // Terminal is Microphone (0x01,0x02)
    0x00,                         // No association
    0x01,                         // One channel
    0x00,0x00,                    // Mono sets no position bits
    0x00,                         // Unused.
    0x00,                         // Unused.
	
		
	// Microphone audio FU (feature unit) descriptor
	0x09,                                 // bLength
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      // bDescriptorType
	AUDIO_CONTROL_FEATURE_UNIT,           // bDescriptorSubtype
	0x05,              						// bUnitID
	0x01,                                 // bSourceID
	0x01,                                 // bControlSize
	AUDIO_CONTROL_VOLUME | AUDIO_CONTROL_MUTE,                   // bmaControls(0)
	0x00,                                 // bmaControls(1)
	0x00,                                 // iTerminal
	

    // USB Microphone Output Terminal Descriptor
    0x09,                            // Size of the descriptor, in bytes (bLength)
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, // CS_INTERFACE Descriptor Type (bDescriptorType)
    AUDIO_CONTROL_OUTPUT_TERMINAL,   // OUTPUT_TERMINAL descriptor subtype (bDescriptorSubtype)
    0x09,                            // ID of this Terminal. (bTerminalID)
    0x01, 0x01,                      // USB Streaming. (wTerminalType
    0x00,                            // unused         (bAssocTerminal)
    0x05,                            // From Audio Control Selector Unit.(bSourceID)
    0x00,                            // unused  (iTerminal)
	
    /* USB Microphone Standard AS Interface Descriptor (Alt. Set. 0) (CODE == 3)*/ //zero-bandwidth interface
    0x09,                         // Size of the descriptor, in bytes (bLength)
    USB_INTERFACE_DESCRIPTOR_TYPE,    // INTERFACE descriptor type (bDescriptorType) 0x04
    0x01, // Index of this interface. (bInterfaceNumber) ?????????? (3<) (1<<) (1<M)
    0x00,                         // Index of this alternate setting. (bAlternateSetting)
    0x00,                         // 0 endpoints.   (bNumEndpoints)
    USB_DEVICE_CLASS_AUDIO,       // AUDIO (bInterfaceClass)
    AUDIO_SUBCLASS_AUDIOSTREAMING, // AUDIO_STREAMING (bInterfaceSubclass)
    0x00,                         // Unused. (bInterfaceProtocol)
    0x00,                         // Unused. (iInterface)

    /* USB Microphone Standard AS Interface Descriptor (Alt. Set. 1) (CODE == 4)*/
    0x09,                         // Size of the descriptor, in bytes (bLength)
    USB_INTERFACE_DESCRIPTOR_TYPE,     // INTERFACE descriptor type (bDescriptorType)
    0x01, // Index of this interface. (bInterfaceNumber)
    0x01,                         // Index of this alternate setting. (bAlternateSetting)
    0x01,                         // 1 endpoint (bNumEndpoints)
    USB_DEVICE_CLASS_AUDIO,       // AUDIO (bInterfaceClass)
    AUDIO_SUBCLASS_AUDIOSTREAMING,   // AUDIO_STREAMING (bInterfaceSubclass)
    0x00,                         // Unused. (bInterfaceProtocol)
    0x00,                         // Unused. (iInterface)

    /*  USB Microphone Class-specific AS General Interface Descriptor (CODE == 5)*/
    0x07,                         // Size of the descriptor, in bytes (bLength)
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, // CS_INTERFACE Descriptor Type (bDescriptorType) 0x24
    AUDIO_STREAMING_GENERAL,         // GENERAL subtype (bDescriptorSubtype) 0x01
    0x09,             // Unit ID of the Output Terminal.(bTerminalLink)
    0x01,                         // Interface delay. (bDelay)
    0x01,0x00,                    // PCM Format (wFormatTag)

    /*  USB Microphone Type I Format Type Descriptor (CODE == 6)*/
    0x0B,                        // Size of the descriptor, in bytes (bLength)
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,// CS_INTERFACE Descriptor Type (bDescriptorType) 0x24
    AUDIO_STREAMING_FORMAT_TYPE,   // FORMAT_TYPE subtype. (bDescriptorSubtype) 0x02
    0x01,                        // FORMAT_TYPE_I. (bFormatType)
    USBD_IN_AUDIO_CH,            // One channel.(bNrChannels)
    0x02,                        // Two bytes per audio subframe.(bSubFrameSize)
    0x10,                        // 16 bits per sample.(bBitResolution)
    0x01,                        // One frequency supported. (bSamFreqType)
    (USBD_IN_AUDIO_FREQ&0xFF),((USBD_IN_AUDIO_FREQ>>8)&0xFF),((USBD_IN_AUDIO_FREQ>>16)&0xFF),  // (tSamFreq)

    /*  USB Microphone Standard Endpoint Descriptor (CODE == 8)*/ //Standard AS Isochronous Audio Data Endpoint Descriptor
    0x09,                       // Size of the descriptor, in bytes (bLength)
    0x05,                       // ENDPOINT descriptor (bDescriptorType)
    AUDIO_IN_EP,                    // IN Endpoint 1. (bEndpointAddress)
    USB_ENDPOINT_TYPE_ISOCHRONOUS, // Isochronous, not shared. (bmAttributes)//USB_ENDPOINT_TYPE_asynchronous USB_ENDPOINT_TYPE_ISOCHRONOUS
    (AUDIO_IN_PACKET&0xFF),((AUDIO_IN_PACKET>>8)&0xFF),                  //bytes per packet (wMaxPacketSize)
    0x01,                       // One packet per frame.(bInterval)
    0x00,                       // Unused. (bRefresh)
    0x00,                       // Unused. (bSynchAddress)

    //  USB Microphone feedback Endpoint Descriptor (CODE == 8) //Standard AS Isochronous Audio Data Endpoint Descriptor
    0x09,                       // Size of the descriptor, in bytes (bLength)
    0x05,                       // ENDPOINT descriptor (bDescriptorType)
    AUDIO_IN_EP+1,                    // IN Endpoint 1. (bEndpointAddress)
    USB_ENDPOINT_TYPE_ISOCHRONOUS, // Isochronous, not shared. (bmAttributes)//USB_ENDPOINT_TYPE_asynchronous USB_ENDPOINT_TYPE_ISOCHRONOUS
    4,0,                  //bytes per packet (wMaxPacketSize)
    0x01,                       // One packet per frame.(bInterval)
    0x00,                       // Unused. (bRefresh)
    0x00,                       // Unused. (bSynchAddress)

    /* USB Microphone Class-specific Isoc. Audio Data Endpoint Descriptor (CODE == 7) OK - подтверждено документацией*/
    0x07,                       // Size of the descriptor, in bytes (bLength)
    AUDIO_ENDPOINT_DESCRIPTOR_TYPE,    // CS_ENDPOINT Descriptor Type (bDescriptorType) 0x25
    AUDIO_ENDPOINT_GENERAL,            // GENERAL subtype. (bDescriptorSubtype) 0x01
    0x00,                              // No sampling frequency control, no pitch control, no packet padding.(bmAttributes)
    0x00,                              // Unused. (bLockDelayUnits)
    0x00,0x00,                         // Unused. (wLockDelay)
};


/**
* @brief  usbd_audio_Init
*         Initilaizes the AUDIO interface.
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  usbd_audio_Init (void  *pdev, 
                                 uint8_t cfgidx)
{  
    /* Open EP IN */
	

  DCD_EP_Open(pdev,
              AUDIO_IN_EP,
              (AUDIO_IN_PACKET),
              USB_OTG_EP_ISOC);

    /* Open feedback EP IN */
  DCD_EP_Open(pdev,
              AUDIO_IN_EP+1,
              (4),
              USB_OTG_EP_ISOC);
	
  /* Initialize the Audio Hardware layer */

  return USBD_OK;
}

/**
* @brief  usbd_audio_DeInit
*         DeInitializes the AUDIO layer.
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  usbd_audio_DeInit (void  *pdev, 
                                   uint8_t cfgidx)
{ 
  DCD_EP_Close (pdev , AUDIO_IN_EP);
  
  /* DeInitialize the Audio output Hardware layer */

  return USBD_OK;
}

/**
  * @brief  usbd_audio_Setup
  *         Handles the Audio control request parsing.
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
	uint8_t buf[10] = {0};
static uint8_t  usbd_audio_Setup (void  *pdev, 
                                  USB_SETUP_REQ *req)
{
  uint16_t len;
  uint8_t  *pbuf;

  int32_t v = 0;
  int16_t *p = (int16_t*)buf;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* AUDIO Class Requests -------------------------------*/
	  // cur, cur, min, max, res
  case USB_REQ_TYPE_CLASS :    
    switch (req->bRequest)
    {
	case 0x82:	// GET_MIN
		v = (-63*0x10);
		*p = v;
		USBD_CtlSendData (pdev, buf, req->wLength);
		break;
	case 0x83:	// GET_MAX
		*p = 0;
		USBD_CtlSendData (pdev, buf, req->wLength);
		break;
	case 0x84:  // GET_RES
 		*p = 0x10;
		USBD_CtlSendData (pdev, buf, req->wLength);
		break;
   case AUDIO_REQ_GET_CUR: //запрос состояния mute
		AUDIO_Req_GetCurrent(pdev, req);
		break;
      
    case AUDIO_REQ_SET_CUR:
		AUDIO_Req_SetCurrent(pdev, req);   
		break;
	  

    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }
    break;
    
    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
      {
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
        pbuf = usbd_audio_Desc;   
#else
        pbuf = usbd_audio_CfgDesc + 18;
#endif 
        len = MIN(USB_AUDIO_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, pbuf, len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev, (uint8_t *)&usbd_audio_AltSet, 1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < AUDIO_TOTAL_IF_NUM)
      {
        usbd_audio_AltSet = (uint8_t)(req->wValue);
        if (usbd_audio_AltSet == 1)
		{
		  PlayFlag = 1;
		} 
		else 
		{ 
        	PlayFlag = 0;
        	DCD_EP_Flush (pdev,AUDIO_IN_EP);
        }
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
  * @brief  usbd_audio_EP0_RxReady
  *         Handles audio control requests data.
  * @param  pdev: device device instance
  * @retval status
  */
static uint8_t  usbd_audio_EP0_RxReady (void  *pdev)
{
	// got control cmd
	if (AudioCtlLen == 2)
		volume = *(int16_t*)AudioCtlBuf;
	if (AudioCtlLen == 1)
		muted = *AudioCtlBuf;
	
	volume_changed = 1;
	
	return USBD_OK;
}

/**
  * @brief  usbd_audio_DataIn
  *         Handles the audio IN data stage.
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
//handle request from HOST
int usb_idle = 1;
uint8_t tx_buf[AUDIO_IN_PACKET];
int do_tx(void *pdev)
{
	// TODO
	int n = codec_pop(tx_buf, AUDIO_IN_PACKET);
	if (n > 0)
	{
		usb_idle = 0;
		DCD_EP_Flush(pdev,AUDIO_IN_EP);
		DCD_EP_Tx (pdev,AUDIO_IN_EP, (uint8_t*)tx_buf, n);
		
		return 0;
	}
	
	
	return 1;
}
static uint8_t  usbd_audio_DataIn (void *pdev, uint8_t epnum)
{  
	usb_idle = 1;
	
	do_tx(pdev);	

	return USBD_OK;
}

/**
  * @brief  usbd_audio_DataOut
  *         Handles the Audio Out data stage.
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_audio_DataOut (void *pdev, uint8_t epnum)
{     
  //for speaker only
  return USBD_OK;
}


/**
  * @brief  usbd_audio_SOF
  *         Handles the SOF event (data buffer update and synchronization).//start-of-frame
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */

int last_pos = -1;
int n = 0;
int samples;
int samples_1s;
static uint8_t  usbd_audio_SOF (void *pdev)
{
	USB_OTG_DSTS_TypeDef  FS_DSTS;
	
	// check adc speed
	/*
	int v = codec_pos();
	int dt = CODEC_DMA_BUF_LEN/2;
	if (last_pos >= 0)
	{
		if (v>= last_pos)
			dt = v - last_pos;
		else
			dt = CODEC_DMA_BUF_LEN + v - last_pos;
		n ++ ;
		samples += dt;
		
		
		if (n == 1000)
		{
			n = 0;
			samples_1s = samples;
			samples = 0;
			GPIO_ToggleBits(GPIOB, GPIO_Pin_15);
		}
	}
	
	
	last_pos = v;
	*/
	
	// Check if there are available data in stream buffer.
	if (PlayFlag == 1) {

		DCD_EP_Tx (pdev,AUDIO_IN_EP, NULL, AUDIO_IN_PACKET);
		PlayFlag = 2;
	}

	//GPIO_ToggleBits(GPIOB, GPIO_Pin_5);
	if (usb_idle && PlayFlag == 2)
	{
		if (do_tx(pdev) == 0)
		{
			n = n;
		}
	}

	return USBD_OK;
}


/******************************************************************************
     AUDIO Class requests management
******************************************************************************/
/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_GetCurrent(void *pdev, USB_SETUP_REQ *req)
{  
  /* Send the current mute state */
  
	if (req->wLength == 1)
		USBD_CtlSendData (pdev, &muted, req->wLength);
	else if (req->wLength == 2)
		USBD_CtlSendData (pdev, &volume, req->wLength);
	else
		USBD_CtlError (pdev, req);
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_SetCurrent(void *pdev, USB_SETUP_REQ *req)
{ 
  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, 
                       AudioCtlBuf,
                       req->wLength);
    
    /* Set the global variables indicating current request and its length 
    to the function usbd_audio_EP0_RxReady() which will process the request */
    AudioCtlCmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    AudioCtlLen = req->wLength;          /* Set the request data length */
    AudioCtlUnit = HIBYTE(req->wIndex);  /* Set the request target unit */
  }
}

/**
  * @brief  USBD_audio_GetCfgDesc 
  *         Returns configuration descriptor.
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_audio_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_audio_CfgDesc);
  return usbd_audio_CfgDesc;
}
/**
  * @}
  */ 

static uint8_t  usbd_audio_OUT_Incplt (void  *pdev)
{
  return USBD_OK;
}

static uint8_t  usbd_audio_IN_Incplt (void  *pdev)
{
  return USBD_OK;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
