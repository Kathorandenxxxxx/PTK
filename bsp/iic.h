#ifndef __IIC_H__
#define __IIC_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"


typedef struct{
    GPIO_TypeDef* SCL_PORT;
    uint16_t SCL_PIN;
    GPIO_TypeDef* SDA_PORT;
    uint16_t SDA_PIN;
}iic_bus_t;

void iic_init(iic_bus_t* bus);
void iic_bus_recovery(iic_bus_t* bus);
void iic_start(iic_bus_t* bus);
void iic_stop(iic_bus_t* bus);
void iic_send_ack(iic_bus_t* bus);
void iic_send_nack(iic_bus_t* bus);
uint8_t iic_wait_ack(iic_bus_t* bus);
void iic_send_byte(iic_bus_t* bus, uint8_t data);
uint8_t iic_read_byte(iic_bus_t* bus);
void SCL_Output(iic_bus_t* bus, uint8_t value);
#ifdef __cplusplus
}
#endif

#endif

