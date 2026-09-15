#ifndef __DELAY_H_
#define __DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"


void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif 
