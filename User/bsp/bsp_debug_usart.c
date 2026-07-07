  
#include "bsp_debug_usart.h"

static UART_HandleTypeDef huart;  

 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */  
void DEBUG_USART_Config(void)
{ 
	
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  
  DEBUG_USART_CLK_ENABLE();
  DEBUG_USART_RX_GPIO_CLK_ENABLE();
  DEBUG_USART_TX_GPIO_CLK_ENABLE();
  
  /**USART1 GPIO Configuration    
  PA9     ------> USART1_TX
  PA10    ------> USART1_RX 
  */
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStruct.Pin   = DEBUG_USART_TX_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed =  GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);
  
  /* 配置Rx引脚为复用功能 */
  GPIO_InitStruct.Pin  = DEBUG_USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;	//模式要设置为复用输入模式！	
  HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct); 
  
  huart.Instance         = DEBUG_USART;
  
  huart.Init.BaudRate    = DEBUG_USART_BAUDRATE;
  huart.Init.WordLength  = UART_WORDLENGTH_8B;
  huart.Init.StopBits    = UART_STOPBITS_1;
  huart.Init.Parity      = UART_PARITY_NONE;
  huart.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
  huart.Init.Mode        = UART_MODE_TX_RX;
  
  HAL_UART_Init(&huart); 
}


//重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&huart, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

//重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{		
	int ch;
	HAL_UART_Receive(&huart, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

/*********************************************END OF FILE**********************/
