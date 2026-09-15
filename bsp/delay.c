#include "delay.h"

//SysTick是一个24位的递减计数器，计数范围为0~0xFFFFFF
void delay_us(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    uint32_t prev = SysTick->VAL;
    uint32_t elapsed = 0;

    // 每次循环累加“上次读取到本次读取”的计数值，能正确处理多次绕回
    while (elapsed < ticks)
    {
        uint32_t now = SysTick->VAL;
        if (now <= prev)
        {
            elapsed += prev - now;                                /* 未绕回 */
        }
        else
        {
            elapsed += prev + (SysTick->LOAD + 1) - now;          /* 绕回一次 */
        }
        prev = now;
    }
}



void delay_ms(uint32_t ms)
{
    uint32_t startTick = HAL_GetTick();
    while(HAL_GetTick() - startTick < ms);
}

