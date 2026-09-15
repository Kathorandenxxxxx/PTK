#include "iic.h"
#include "delay.h"

static void SDA_InMode(iic_bus_t* bus)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = bus->SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(bus->SDA_PORT, &GPIO_InitStruct);
}

static void SDA_OutMode(iic_bus_t* bus)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = bus->SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* 必须设置！否则 MODE=00=Input */
    HAL_GPIO_Init(bus->SDA_PORT, &GPIO_InitStruct);
}

void SCL_Output(iic_bus_t* bus, uint8_t value)
{
    //操作寄存器
    bus->SCL_PORT->BSRR = (value ? bus->SCL_PIN : ((uint32_t)bus->SCL_PIN << 16U));
}

static void SDA_Output(iic_bus_t* bus, uint8_t value)
{
    //操作寄存器
    bus->SDA_PORT->BSRR = (value ? bus->SDA_PIN : ((uint32_t)bus->SDA_PIN << 16U));
}

static uint8_t SDA_Input(iic_bus_t* bus)
{
    return ((bus->SDA_PORT->IDR & bus->SDA_PIN) ? 1 : 0);
}

void iic_init(iic_bus_t* bus)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = bus->SCL_PIN | bus->SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* 必须设置！否则 MODE=00=Input */

    HAL_GPIO_Init(bus->SDA_PORT, &GPIO_InitStruct);

    // 初始电平，总线空闲
    SDA_Output(bus, 1);
    SCL_Output(bus, 1);
    delay_us(1);
}

/*===========================================================================
 * I2C 总线恢复 (释放被从机锁死的 SDA)
 * 场景: MCU 复位时 I2C 事务被打断, 从机可能一直拉低 SDA 锁死总线,
 *       导致后续 START/寻址全部 NACK。连续给 9 个 SCL 时钟脉冲,
 *       让从机把当前字节剩余位发完并释放 SDA, 再补一个 STOP 收尾。
 *===========================================================================*/
void iic_bus_recovery(iic_bus_t* bus)
{
    uint8_t i;

    /* 释放 SDA (主机只读不驱动, 靠上拉决定电平), SCL 由主机控制 */
    SDA_InMode(bus);

    /* 连续 9 个 SCL 脉冲: 时钟出从机卡住的剩余位, 使其释放 SDA */
    for (i = 0; i < 9; i++)
    {
        SCL_Output(bus, 0);
        delay_us(1);
        SCL_Output(bus, 1);
        delay_us(1);
    }

    /* 补一个 STOP (SCL 高时 SDA 低→高), 结束任何残留传输 */
    SDA_OutMode(bus);
    SDA_Output(bus, 0);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SDA_Output(bus, 1);
    delay_us(1);

    /* 回到空闲状态 */
    SCL_Output(bus, 0);
    delay_us(1);
}

void iic_start(iic_bus_t* bus)
{
    SDA_OutMode(bus);
    SDA_Output(bus, 1);
    delay_us(1);

    SCL_Output(bus, 1);
    delay_us(1);

    SDA_Output(bus, 0);
    delay_us(1);
    
    SCL_Output(bus, 0);
    delay_us(1);
}

void iic_stop(iic_bus_t* bus)
{
    SDA_OutMode(bus);

    SCL_Output(bus, 0);
    delay_us(1);

    SDA_Output(bus, 0);
    delay_us(1);
    
    SCL_Output(bus, 1);
    delay_us(1);
    
    SDA_Output(bus, 1);
    delay_us(1);
}
void iic_send_nack(iic_bus_t* bus)
{
    SDA_OutMode(bus);
    SDA_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 0);
    delay_us(1);
}

void iic_send_ack(iic_bus_t* bus)
{
    SDA_OutMode(bus);
    SDA_Output(bus, 0);
    delay_us(1);
    SCL_Output(bus, 1);
    delay_us(1);
    SCL_Output(bus, 0);
    delay_us(1);
}

/** @brief  Wait for ACK
  * @param  bus: IIC bus
  * @retval ACK status 0 = ACK, 1 = NACK 
  */
uint8_t iic_wait_ack(iic_bus_t* bus)
{
    SDA_InMode(bus);
    SCL_Output(bus, 1);
    delay_us(1);
    uint8_t ack = SDA_Input(bus);
    SCL_Output(bus, 0);
    delay_us(1);
    return ack;
}


void iic_send_byte(iic_bus_t* bus, uint8_t data)
{
    SDA_OutMode(bus);
    //高位先出
    for(uint8_t i = 0; i < 8; i++)
    {
        SCL_Output(bus, 0);
        delay_us(1);
        if(data & 0x80)
        {
            SDA_Output(bus, 1);
        }
        else
        {
            SDA_Output(bus, 0);
        }
        delay_us(1);
        SCL_Output(bus, 1); 
        delay_us(1);
        data <<= 1;
    }
    SCL_Output(bus, 0);
//    delay_us(1);
}

uint8_t iic_read_byte(iic_bus_t* bus)
{
    SDA_InMode(bus);
    //先读高位
    uint8_t data = 0;
    for(uint8_t i=0; i<8; i++)
    {
        SCL_Output(bus, 0);
        delay_us(1);
        SCL_Output(bus, 1);
        delay_us(1);
        uint8_t bit = SDA_Input(bus);
        data = (data << 1) | bit;
    }
    SCL_Output(bus, 0);
    delay_us(1);
    return data;
}

