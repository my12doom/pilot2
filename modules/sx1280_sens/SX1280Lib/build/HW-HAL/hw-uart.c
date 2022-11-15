#include "hw.h"

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar( int ch )
#else
#define PUTCHAR_PROTOTYPE int fputc( int ch, FILE *f )
#endif /* __GNUC__ */

UART_HandleTypeDef UartHandle;

void UartInit( void )
{
    UartHandle.Instance                     = USART2;
    UartHandle.Init.BaudRate                = 115200;
    UartHandle.Init.WordLength              = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits                = UART_STOPBITS_1;
    UartHandle.Init.Parity                  = UART_PARITY_NONE;
    UartHandle.Init.Mode                    = UART_MODE_TX_RX;
    UartHandle.Init.HwFlowCtl               = UART_HWCONTROL_NONE;
    UartHandle.Init.OverSampling            = UART_OVERSAMPLING_16;
    UartHandle.Init.OneBitSampling          = UART_ONE_BIT_SAMPLE_DISABLE;
    UartHandle.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_NO_INIT;

    if( HAL_UART_Init( &UartHandle ) != HAL_OK )
    {
        Error_Handler( );
    }
}

void UartDeInit( void )
{
    HAL_UART_DeInit( &UartHandle );
}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit( &UartHandle, ( uint8_t * )&ch, 1, 0xFFFF );

  return ch;
}

#ifdef USE_FULL_ASSERT
/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed( uint8_t* file, uint32_t line )
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif
