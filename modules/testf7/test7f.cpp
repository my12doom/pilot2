//#include <HAL/STM32F4/F4UART.h>
//#include <HAL/Interface/ISysTimer.h>
//#include <FileSystem/ff.h>
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <HAL/STM32F7/F7SysTimer.h>
#include <HAL/STM32F7/F7GPIO.h>
#include <HAL/Interface/Interfaces.h>
#include <Algorithm\ekf_estimator.h>
//// BSP
using namespace STM32F7_HAL;
using namespace HAL;
//F4UART uart1(USART1);

//extern "C" void USART1_IRQHandler(void)
//{
//	uart1.USART1_IRQHandler();
//}
//extern "C" void DMA2_Stream7_IRQHandler()
//{
//	uart1.DMA2_Steam7_IRQHandler();
//}
//int main()
//{


//}
//	

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 270;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPDIFRX;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = 0;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOI_CLK_ENABLE();


   /*Configure GPIO pin : PI1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

}

void delay(volatile int p)
{
	while(p--)
		;
}

/* ADC1 init function */
ADC_HandleTypeDef hadc1;
void MX_ADC1_Init(void)
{
	__ADC1_CLK_ENABLE();
	
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

int main()
{
	SCB_EnableICache();
	SCB_EnableDCache();
	
  HAL_Init();
	
  SystemClock_Config();
	
  ((F7SysTimer*)systimer)->config();

  //MX_GPIO_Init();
  MX_ADC1_Init();
  F7GPIO led(GPIOI,GPIO_PIN_1);
  led.set_mode(MODE_OUT_PushPull);

  ekf_estimator ekf;
  ekf.init(0,0,1, 1,0,0, 0,0,0);
	
  while (1)
  {
	  int64_t start = systimer->gettime();
	  EKF_U U = {0,0,0,0,0,1, };
	  EKF_Mesurement mess = {1,0,0, 0,0,0, 0,0, 0,0,0, 0,0,0};
	  ekf.update(U, mess, 0.003f);
	  
	  start = systimer->gettime() - start;
	  
	  HAL_ADC_Start(&hadc1);
	  while(!(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)))
		  ;
	  float vsense = HAL_ADC_GetValue(&hadc1) * 3.30 / 4095;
	  float v25 = 0.76f;
	  float avg_slope = 0.0025f;
	  float temp = (vsense - v25) / avg_slope + 25;

	  
	  printf("\r%dus, temp=%.2f,     ", int(start), temp);
	  
	  static int64_t last_time = 0;
	  if (systimer->gettime() - last_time > 1000000)
	  {
		  last_time = systimer->gettime();
		  
		  printf("\n");
	  }
	  
	  /*
	  led.write(true);
	  systimer->delayms(1000);
	  led.write(false);
	  systimer->delayms(1000);
	  */
	  
  }
}
	

struct __FILE { int handle; /* Add whatever you need here */ };
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

extern "C" int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) 
	{
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return (ch);
}
