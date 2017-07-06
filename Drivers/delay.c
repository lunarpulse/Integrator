#include <delay.h>

extern volatile uint32_t ticker;

void SysTick_Init(void)
{
 if (SysTick_Config (SystemCoreClock / 1000)) //1ms per interrupt
  while (1);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_SetPriority(SysTick_IRQn,0);
  ticker = 0;
}

uint32_t generated_ms(void)
{
  return ticker;
}

uint32_t generated_us(void)
{
   /* us = ticker*1000+(SystemCoreClock/1000-SysTick->VAL)/80; */
   return ticker * 1000 + 1000 - SysTick->VAL/(SystemCoreClock/1000000UL);
}

void delay_ms(uint32_t nTime)
{
 uint32_t curTime = ticker;
  while((nTime-(ticker-curTime)) > 0);
}

void delay_us(uint32_t nTime)
{
 uint32_t curTime = generated_us();
 while((nTime-(generated_us()-curTime)) > 0);
}