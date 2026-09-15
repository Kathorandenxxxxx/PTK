#ifndef __MPU6050_H__
#define __MPU6050_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "iic.h"

/*===========================================================================
 * I2C 地址
 *===========================================================================*/
#define MPU6050_I2C_ADDR          0x68   /* AD0=GND: 7位地址, 写=0xD0, 读=0xD1 */
#define MPU6050_I2C_WRITE_ADDR    (MPU6050_I2C_ADDR << 1)
#define MPU6050_I2C_READ_ADDR     ((MPU6050_I2C_ADDR << 1) | 1)

/*===========================================================================
 * 寄存器地址
 *===========================================================================*/

/*----- Chip ID -----*/
#define REG_WHO_AM_I              0x75    /* Who Am I (预期: 0x68) */
#define WHO_AM_I_VAL              0x68

/*----- Power Management -----*/
#define REG_PWR_MGMT_1            0x6B    /* 电源管理1: 睡眠/时钟源/温度传感器 */
#define REG_PWR_MGMT_2            0x6C    /* 电源管理2: 各轴加速度计唤醒 */

/*----- Sample Rate -----*/
#define REG_SMPLRT_DIV            0x19    /* 采样率分频: 采样率=内部频率/(1+DIV) */

/*----- DLPF Config -----*/
#define REG_CONFIG                0x1A    /* 数字低通滤波器配置 */

/*----- Gyroscope Config -----*/
#define REG_GYRO_CONFIG           0x1B    /* 陀螺仪满量程选择 */

/*----- Accelerometer Config -----*/
#define REG_ACCEL_CONFIG          0x1C    /* 加速度计满量程选择 */

/*----- FIFO -----*/
#define REG_FIFO_EN               0x23    /* FIFO 使能控制 */

/*----- Interrupt -----*/
#define REG_INT_PIN_CFG           0x37    /* INT 引脚配置 */
#define REG_INT_ENABLE            0x38    /* 中断使能 */
#define REG_INT_STATUS            0x3A    /* 中断状态 */

/*----- Accelerometer Data (3×16bit, Big-Endian) -----*/
#define REG_ACCEL_XOUT_H          0x3B    /* 加速度 X 轴高字节 */
#define REG_ACCEL_XOUT_L          0x3C
#define REG_ACCEL_YOUT_H          0x3D
#define REG_ACCEL_YOUT_L          0x3E
#define REG_ACCEL_ZOUT_H          0x3F
#define REG_ACCEL_ZOUT_L          0x40

/*----- Temperature (16bit, Big-Endian) -----*/
#define REG_TEMP_OUT_H            0x41    /* 温度高字节 */
#define REG_TEMP_OUT_L            0x42

/*----- Gyroscope Data (3×16bit, Big-Endian) -----*/
#define REG_GYRO_XOUT_H           0x43    /* 陀螺仪 X 轴高字节 */
#define REG_GYRO_XOUT_L           0x44
#define REG_GYRO_YOUT_H           0x45
#define REG_GYRO_YOUT_L           0x46
#define REG_GYRO_ZOUT_H           0x47
#define REG_GYRO_ZOUT_L           0x48

/*----- User Control -----*/
#define REG_USER_CTRL             0x6A    /* 用户控制: FIFO/I2C主模式/复位 */

/*----- Signal Path Reset -----*/
#define REG_SIGNAL_PATH_RESET     0x68    /* 信号通路复位 */

/*===========================================================================
 * 位域宏
 *===========================================================================*/

/* PWR_MGMT_1 (0x6B) */
#define PWR_MGMT1_DEVICE_RESET    0x80    /* 复位所有寄存器 (自动清零) */
#define PWR_MGMT1_SLEEP           0x40    /* 1=睡眠模式, 0=唤醒 */
#define PWR_MGMT1_CYCLE           0x20    /* 1=循环唤醒模式 */
#define PWR_MGMT1_TEMP_DIS        0x08    /* 1=关闭温度传感器 */
#define PWR_MGMT1_CLKSEL_MASK     0x07    /* 时钟源选择掩码 */
#define PWR_MGMT1_CLKSEL_INTERNAL 0x00    /* 内部 8MHz 振荡器 */
#define PWR_MGMT1_CLKSEL_PLL_X    0x01    /* PLL (X轴陀螺仪参考) */
#define PWR_MGMT1_CLKSEL_PLL_Y    0x02    /* PLL (Y轴陀螺仪参考) */
#define PWR_MGMT1_CLKSEL_PLL_Z    0x03    /* PLL (Z轴陀螺仪参考) */
#define PWR_MGMT1_CLKSEL_EXT_32K  0x04    /* 外部 32.768kHz */
#define PWR_MGMT1_CLKSEL_EXT_19M  0x05    /* 外部 19.2MHz */
#define PWR_MGMT1_CLKSEL_STOP     0x07    /* 停止时钟 */

/* CONFIG (0x1A) — DLPF 数字低通滤波器 */
/* [2:0] DLPF_CFG: 带宽 → 陀螺仪采样率 / 加速度计采样率 */
#define DLPF_CFG_256HZ            0x00    /* 8kHz 内部采样, 256Hz 带宽 */
#define DLPF_CFG_188HZ            0x01    /* 1kHz 内部采样, 188Hz 带宽 */
#define DLPF_CFG_98HZ             0x02    /* 1kHz 内部采样,  98Hz 带宽 */
#define DLPF_CFG_42HZ             0x03    /* 1kHz 内部采样,  42Hz 带宽 (推荐) */
#define DLPF_CFG_20HZ             0x04    /* 1kHz 内部采样,  20Hz 带宽 */
#define DLPF_CFG_10HZ             0x05    /* 1kHz 内部采样,  10Hz 带宽 */
#define DLPF_CFG_5HZ              0x06    /* 1kHz 内部采样,   5Hz 带宽 */

/* GYRO_CONFIG (0x1B) — 陀螺仪满量程 */
/* [4:3] FS_SEL: 量程范围 */
#define GYRO_FS_SEL_250           0x00    /* ±250 °/s,  131.0 LSB/(°/s) */
#define GYRO_FS_SEL_500           0x08    /* ±500 °/s,   65.5 LSB/(°/s) */
#define GYRO_FS_SEL_1000          0x10    /* ±1000 °/s,  32.8 LSB/(°/s) */
#define GYRO_FS_SEL_2000          0x18    /* ±2000 °/s,  16.4 LSB/(°/s) */

/* ACCEL_CONFIG (0x1C) — 加速度计满量程 */
/* [4:3] AFS_SEL: 量程范围 */
#define ACCEL_FS_SEL_2G           0x00    /* ±2g,  16384 LSB/g */
#define ACCEL_FS_SEL_4G           0x08    /* ±4g,   8192 LSB/g */
#define ACCEL_FS_SEL_8G           0x10    /* ±8g,   4096 LSB/g */
#define ACCEL_FS_SEL_16G          0x18    /* ±16g,  2048 LSB/g */

/* INT_PIN_CFG (0x37) */
#define INT_PIN_CFG_INT_LEVEL     0x80    /* 1=高电平有效, 0=低电平有效 */
#define INT_PIN_CFG_INT_OPEN      0x40    /* 1=开漏输出, 0=推挽输出 */
#define INT_PIN_CFG_LATCH_INT_EN  0x20    /* 1=锁存中断(读STATUS后清除) */
#define INT_PIN_CFG_INT_RD_CLEAR  0x10    /* 1=读数据清除中断 */

/* INT_ENABLE (0x38) */
#define INT_ENABLE_DATA_RDY       0x01    /* 数据就绪中断 */

/* USER_CTRL (0x6A) */
#define USER_CTRL_FIFO_EN         0x40    /* FIFO 使能 */
#define USER_CTRL_I2C_MST_EN      0x20    /* I2C 主模式使能 */
#define USER_CTRL_I2C_IF_DIS      0x10    /* 1=关闭I2C(仅SPI可用) */
#define USER_CTRL_FIFO_RESET      0x04    /* FIFO 复位 */
#define USER_CTRL_I2C_MST_RESET   0x02    /* I2C 主模式复位 */
#define USER_CTRL_SIG_COND_RESET  0x01    /* 信号通路复位 */

/*===========================================================================
 * 引脚定义
 *===========================================================================*/
#define MPU6050_SCL_PORT          GPIOB
#define MPU6050_SCL_PIN           GPIO_PIN_0
#define MPU6050_SDA_PORT          GPIOB
#define MPU6050_SDA_PIN           GPIO_PIN_1

/*===========================================================================
 * 量程转换因子
 *===========================================================================*/
/* 当前配置: ±4g / ±500°/s */
#define ACCEL_SCALE_4G            8192.0f   /* LSB/g  (4096×2 / 1g) */
#define GYRO_SCALE_500            65.5f     /* LSB/(°/s)  (32768 / 500) */
#define RAD_TO_DEG                57.29578f   /* 1 rad = 57.29577951308232 ° */
#define DEG_TO_RAD                0.0174533f  /* 1 ° = 0.0174532925199433 rad */

/*===========================================================================
 * 姿态角结构体
 *===========================================================================*/
typedef struct {
    int16_t gyro_x;   /* 陀螺仪x轴角速度 (°/s) */
    int16_t gyro_y;   /* 陀螺仪y轴角速度 (°/s) */
    int16_t gyro_z;   /* 陀螺仪z轴角速度 (°/s) */
    int16_t accel_x;  /* 加速度计x轴加速度 (g) */
    int16_t accel_y;  /* 加速度计y轴加速度 (g) */
    int16_t accel_z;  /* 加速度计z轴加速度 (g) */
    int16_t gx_offset;  /* 陀螺仪x轴零偏 (LSB) */
    int16_t gy_offset;  /* 陀螺仪y轴零偏 (LSB) */
    int16_t gz_offset;  /* 陀螺仪z轴零偏 (LSB) */
    
    float pitch;      /* 俯仰角 (°) */
    float roll;       /* 横滚角 (°) */
    float yaw;        /* 偏航角 (°) */
    uint32_t last_time;  /* 上次更新时间戳 (ms) */
} mpu6050_attitude_t;

/*===========================================================================
 * 全局变量
 *===========================================================================*/
extern iic_bus_t mpu6050_iic_bus;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/*----- 底层 I2C 通信 -----*/
void    mpu6050_GPIO_Init(void);
uint8_t mpu6050_WriteReg(uint8_t reg, uint8_t data);
uint8_t mpu6050_ReadReg(uint8_t reg);
uint8_t mpu6050_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len);

/*----- 初始化 -----*/
uint8_t MPU6050_Init(void);

/*----- 单位转换 -----*/
float   MPU6050_GyroToDPS(int16_t raw);    /* LSB → °/s */
float   MPU6050_AccelToG(int16_t raw);     /* LSB → g */

/*----- 原始数据读取 -----*/
void    MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
void    MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);
int16_t MPU6050_ReadTemp(void);

uint8_t MPU6050_Calibrate_GyroAcc(mpu6050_attitude_t *att);  /* 陀螺仪和加速度计零偏校准 (需保持静止) */
/*----- 姿态解算 (DCM 互补滤波) -----*/
// void    MPU6050_AttitudeUpdate(mpu6050_attitude_t *att);
void    MPU6050_KalmanUpdate(mpu6050_attitude_t *att);
#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
