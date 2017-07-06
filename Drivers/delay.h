#ifndef DELAY_H
  #define DELAY_H
  
#include <stm32f4xx.h>
#include <misc.h>

extern void SysTick_Init(void);
extern void delay_ms(uint32_t nTime);
extern void delay_us(uint32_t nTime);
#endif

/*

volatile uint32_t ticker;

void SysTick_Handler(void)
{
  ticker++;
} 
*/ 