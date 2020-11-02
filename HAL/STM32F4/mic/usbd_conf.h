

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

/* Includes ------------------------------------------------------------------*/

#define USBD_IN_AUDIO_FREQ                (48000) //MIC frequency in Hz.
#define USBD_IN_AUDIO_CH                (2) //MIC channels


#define DEFAULT_VOLUME                  80    /* Default volume in % (Mute=0%, Max = 100%) in Logarithmic values.
                                                 To get accurate volume variations, it is possible to use a logarithmic
                                                 coversion table to convert from percentage to logarithmic law.
                                                 In order to keep this example code simple, this conversion is not used.*/
#define AUDIO_TOTAL_IF_NUM              0x02
#define USBD_CFG_MAX_NUM                1
#define USBD_ITF_MAX_NUM                1
#define USB_MAX_STR_DESC_SIZ            200 
#define AUDIO_IN_EP                     0x81

#endif //__USBD_CONF__H__

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

