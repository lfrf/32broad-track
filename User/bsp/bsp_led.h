#ifndef __LED_H
#define	__LED_H

#include "stm32f1xx.h"

//Òý½Å¶¨Òå
/*******************************************************/

#define LED_PIN                  GPIO_PIN_2               
#define LED_GPIO_PORT            GPIOC                    
#define LED_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()

/************************************************************/								

void LED_GPIO_Config(void);

#endif /* __LED_H */
