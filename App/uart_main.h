#ifndef __UART_MAIN_H
#define __UART_MAIN_H


#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_AF7_USART6        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */

      
#define GPIO_PIN_9_SEL                       9
#define GPIO_PIN_10_SEL                      10      
#define GPIO_PIN_5_SEL                       5
#define GPIO_PIN_6_SEL                       6              
#define GPIO_PIN_8_SEL                       8
#define GPIO_PIN_9_SEL                       9             
#define GPIO_PIN_14_SEL                      14

/* Definition for USART1 Pins */
#define USART1_TX_PIN                    GPIO_PIN_9_SEL
#define USART1_TX_GPIO_PORT              GPIOA  
#define USART1_TX_AF                     GPIO_AF7_USART1
#define USART1_RX_PIN                    GPIO_PIN_10_SEL
#define USART1_RX_GPIO_PORT              GPIOA 
#define USART1_RX_AF                     GPIO_AF7_USART1
/* Definition for USART2 Pins */
#define USART2_TX_PIN                    GPIO_PIN_5_SEL
#define USART2_TX_GPIO_PORT              GPIOD  
#define USART2_TX_AF                     GPIO_AF7_USART2
#define USART2_RX_PIN                    GPIO_PIN_6_SEL
#define USART2_RX_GPIO_PORT              GPIOD 
#define USART2_RX_AF                     GPIO_AF7_USART2
/* Definition for USART3 Pins */
#define USART3_TX_PIN                    GPIO_PIN_8_SEL
#define USART3_TX_GPIO_PORT              GPIOD  
#define USART3_TX_AF                     GPIO_AF7_USART3
#define USART3_RX_PIN                    GPIO_PIN_9_SEL
#define USART3_RX_GPIO_PORT              GPIOD 
#define USART3_RX_AF                     GPIO_AF7_USART3
/* Definition for USART6 Pins */
#define USART6_TX_PIN                    GPIO_PIN_14_SEL
#define USART6_TX_GPIO_PORT              GPIOG  
#define USART6_TX_AF                     GPIO_AF7_USART6
#define USART6_RX_PIN                    GPIO_PIN_9_SEL
#define USART6_RX_GPIO_PORT              GPIOG 
#define USART6_RX_AF                     GPIO_AF7_USART6
/* Definition for USART2's NVIC */
#define USART2_IRQn                      USART2_IRQn
#define USART2_IRQHandler                USART2_IRQHandler
/* Definition for USART1's NVIC */
#define USART1_IRQn                      USART1_IRQn
#define USART1_IRQHandler                USART1_IRQHandler
/* Definition for USART3's NVIC */
#define USART3_IRQn                      USART3_IRQn
#define USART3_IRQHandler                USART3_IRQHandler
/* Definition for USART6's NVIC */
#define USART6_IRQn                      USART6_IRQn
#define USART6_IRQHandler                USART6_IRQHandler

#define EXTIx_IRQn                 EXTI0_IRQn
#define EXTIx_IRQHandler           EXTI0_IRQHandler

#define GPIO_BUTTON_PIN   0
#define GPIO_BUTTON_PORT  GPIOA




#endif
