#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f1xx.h"
#include <stdio.h>

//串口波特率
#define DEBUG_USART_BAUDRATE                    115200

//引脚定义
/*******************************************************/
#define DEBUG_USART                             USART1
#define DEBUG_USART_CLK_ENABLE()                __HAL_RCC_USART1_CLK_ENABLE();

#define DEBUG_USART_RX_GPIO_PORT                GPIOA
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_RX_PIN                      GPIO_PIN_10

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_TX_PIN                      GPIO_PIN_9

/************************************************************/

void DEBUG_USART_Config(void);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
#endif /* __USART1_H */
