#ifndef __OLED_H
#define __OLED_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#define OLED_I2C_ADDRESS        0x78 // OLED I2C地址
#define OLED_I2C_WRITE_ADDR     (OLED_I2C_ADDRESS << 1) // OLED I2C写地址
#define OLED_I2C_READ_ADDR      ((OLED_I2C_ADDRESS << 1) | 1) // OLED I2C读地址

/* 引脚定义*/
#define OLED_SCL_PORT GPIOB
#define OLED_SCL_PIN  GPIO_PIN_10
#define OLED_SDA_PORT GPIOB
#define OLED_SDA_PIN  GPIO_PIN_11

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#ifdef __cplusplus
}
#endif

#endif
