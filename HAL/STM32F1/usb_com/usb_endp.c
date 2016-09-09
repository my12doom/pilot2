/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.1.1
* Date               : 04/07/2010
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t buffer_out[64];
__IO uint32_t count_out = 0;
#define tx_packet_size 63

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
}

extern int parse_command_line(const char *line, char *out);

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}

char response[512];

static char* parse_command(char *cmd)
{
	int response_size;
	int pos = 0;
	do
	{
		char *next = (char*)strchr(cmd, '\n');
		if (next)
			next[0] = 0;

		if (NULL == cmd[0])
			break;
		
		response_size = parse_command_line(cmd, response);
		response[response_size] = NULL;
		
		/* Write the data to the USB endpoint */
		//printf("cmd:%s, response: %s(%d byte)\n", cmd, response, response_size);
		
		if (response_size > 128)
		{
			//response[0] = '\n';
			//response_size = 1;
			//printf("oops(avg=%d):%s\n", avg_count, response);
		}
	
		while(pos < response_size)
		{
			USB_SIL_Write(EP1_IN, (uint8_t*)response + pos, min(response_size - pos, tx_packet_size));
			pos += tx_packet_size;
		
#ifndef STM32F10X_CL
			SetEPTxValid(ENDP1);
#endif /* STM32F10X_CL */
			
			while (GetEPTxStatus(ENDP1) == EP_TX_VALID);
		}
		
		if (!next)
			break;
		cmd = next+1;
		
		
	}while (1);

	return cmd;
}

/*******************************************************************************
* Function Name  : EP3_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
	char *end;
	int i;
	int count;
	
	// if we are near buffer overrun, discard all packets
	if (count_out + 64 > sizeof(buffer_out))
	{
		printf("buffer full\n");
		count_out = 0;
	}
	
  // Get the received data buffer and update the counter
	count = USB_SIL_Read(EP3_OUT, buffer_out+count_out);
	
	// replace '\r' into '\n"
	for(i=count_out; i<count_out+count; i++)
		buffer_out[i] = buffer_out[i] == '\r' ? '\n' : buffer_out[i];
	
  count_out += count;
	buffer_out[count_out] = NULL;
	
#ifndef STM32F10X_CL
  // Enable the receive of data on EP3
  SetEPRxValid(ENDP3);
#endif /* STM32F10X_CL */
	
	
	// line handling and coping left to head of buffer
	end = parse_command((char*)buffer_out);
	if (end > (char*)buffer_out)
	{
		int left = strlen(end);
		memmove(buffer_out, end, left+1);
		count_out = left;
		
		// one command each line...
		//printf("cmd handled\n");
		//count_out = 0;
	}
	
	count_out = 0;
}


void EP4_IN_Callback(void)
{
}
void EP6_OUT_Callback(void)
{
}
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

