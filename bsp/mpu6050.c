/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "iic.h"
#include "delay.h"
#include <math.h>

/*===========================================================================
 * I2C 总线实例 (PB0=SCL, PB1=SDA)
 *===========================================================================*/
iic_bus_t mpu6050_iic_bus = {
    .SCL_PORT = MPU6050_SCL_PORT,
    .SCL_PIN  = MPU6050_SCL_PIN,
    .SDA_PORT = MPU6050_SDA_PORT,
    .SDA_PIN  = MPU6050_SDA_PIN
};

/*===========================================================================
 * GPIO 初始化 (I2C 总线)
 *===========================================================================*/
void mpu6050_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    iic_init(&mpu6050_iic_bus);
}

/*===========================================================================
 * I2C 写寄存器
 * 协议: START | WRITE_ADDR | RegAddr | Data | STOP
 * 返回: 0=成功, 1=某步NACK
 *===========================================================================*/
uint8_t mpu6050_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t err = 0;

    iic_start(&mpu6050_iic_bus);

    iic_send_byte(&mpu6050_iic_bus, MPU6050_I2C_WRITE_ADDR);
    err |= iic_wait_ack(&mpu6050_iic_bus);

    iic_send_byte(&mpu6050_iic_bus, reg);
    err |= iic_wait_ack(&mpu6050_iic_bus);

    iic_send_byte(&mpu6050_iic_bus, data);
    err |= iic_wait_ack(&mpu6050_iic_bus);

    iic_stop(&mpu6050_iic_bus);

    return err;   /* 0=OK, 1=NACK */
}

/*===========================================================================
 * I2C 读单字节 (标准 I2C 重复起始)
 * 协议: START | WRITE_ADDR | RegAddr | REPEATED_START | READ_ADDR | Read(NACK) | STOP
 * 返回: 读到的数据字节; 若通信失败返回 0x00 (调用者应先用 WHO_AM_I 验证链路)
 *===========================================================================*/
uint8_t mpu6050_ReadReg(uint8_t reg)
{
    uint8_t data;

    /* Phase 1: 发送寄存器地址 */
    iic_start(&mpu6050_iic_bus);
    iic_send_byte(&mpu6050_iic_bus, MPU6050_I2C_WRITE_ADDR);
    if (iic_wait_ack(&mpu6050_iic_bus))
    {
        iic_stop(&mpu6050_iic_bus);
        return 0x00;   /* NACK on write address */
    }
    iic_send_byte(&mpu6050_iic_bus, reg);
    if (iic_wait_ack(&mpu6050_iic_bus))
    {
        iic_stop(&mpu6050_iic_bus);
        return 0x00;   /* NACK on reg address */
    }

    /* Phase 2: 重复起始 + 读数据 (标准I2C, 非SCCB两段式) */
    iic_start(&mpu6050_iic_bus);                           /* 重复起始, 不先发STOP */
    iic_send_byte(&mpu6050_iic_bus, MPU6050_I2C_READ_ADDR);
    if (iic_wait_ack(&mpu6050_iic_bus))
    {
        iic_stop(&mpu6050_iic_bus);
        return 0x00;   /* NACK on read address */
    }
    data = iic_read_byte(&mpu6050_iic_bus);
    iic_send_nack(&mpu6050_iic_bus);                       /* 主机发NACK表示读结束 */
    iic_stop(&mpu6050_iic_bus);

    return data;
}

/*===========================================================================
 * I2C 连续读多字节
 * 协议: START | WRITE_ADDR | RegAddr | REPEATED_START | READ_ADDR |
 *        Read[0](ACK) | Read[1](ACK) | ... | Read[N-1](NACK) | STOP
 * 用于加速度计/陀螺仪 6 字节连续读取, 减少 I2C 事务次数
 *===========================================================================*/
uint8_t mpu6050_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len == 0) return 0;

    /* Phase 1: 发送起始寄存器地址 */
    iic_start(&mpu6050_iic_bus);
    iic_send_byte(&mpu6050_iic_bus, MPU6050_I2C_WRITE_ADDR);
    if (iic_wait_ack(&mpu6050_iic_bus))
    {
        iic_stop(&mpu6050_iic_bus);
        return 1;
    }
    iic_send_byte(&mpu6050_iic_bus, reg);
    if (iic_wait_ack(&mpu6050_iic_bus))
    {
        iic_stop(&mpu6050_iic_bus);
        return 1;
    }

    /* Phase 2: 重复起始 + 连续读 */
    iic_start(&mpu6050_iic_bus);
    iic_send_byte(&mpu6050_iic_bus, MPU6050_I2C_READ_ADDR);
    if (iic_wait_ack(&mpu6050_iic_bus))
    {
        iic_stop(&mpu6050_iic_bus);
        return 1;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = iic_read_byte(&mpu6050_iic_bus);
        if (i < (len - 1))
            iic_send_ack(&mpu6050_iic_bus);     /* 继续: 发ACK */
        else
            iic_send_nack(&mpu6050_iic_bus);    /* 结束: 发NACK */
    }

    iic_stop(&mpu6050_iic_bus);
    return 0;   /* OK */
}

/*===========================================================================
 * MPU6050 初始化
 * 返回: 0=成功, 1=芯片未检测到
 *===========================================================================*/
uint8_t MPU6050_Init(void)
{
    /* 1. 初始化 I2C 总线 GPIO */
    mpu6050_GPIO_Init();

    /* 总线恢复: 复位时 I2C 事务可能被打断, 从机会锁住 SDA,
     * 先给 9 个时钟脉冲释放总线, 否则后续所有读写都 NACK */
    // iic_bus_recovery(&mpu6050_iic_bus);

    /* 软件复位 MPU6050: 清除上次运行遗留的内部状态
     * (GY-521 无外部复位脚, 只能 I2C 软件复位或断电重插) */
    mpu6050_WriteReg(REG_PWR_MGMT_1, PWR_MGMT1_DEVICE_RESET);
    delay_ms(100);

    /* 2. 验证芯片身份: 读 WHO_AM_I */
    uint8_t id;
    uint8_t retry_times = 3;
    while (retry_times--)
    {
        id = mpu6050_ReadReg(REG_WHO_AM_I);
        if (id != WHO_AM_I_VAL)
        {
            if (retry_times == 0)
            {
                return 1;   /* 芯片未检测到 */
            }
        }
        else
        {
            break;  /* 检测到芯片 */
        }
        delay_ms(100);
    }

    /* 3. 唤醒设备: PWR_MGMT_1 = 0x00 (退出睡眠, 选择内部8MHz振荡器) */
    mpu6050_WriteReg(REG_PWR_MGMT_1, PWR_MGMT1_CLKSEL_INTERNAL);
    delay_ms(10);

    /* 4. 采样率分频器: SMPLRT_DIV = 0 → 采样率 = 1kHz (内部8kHz / 1)
     *    配合 DLPF_CFG=3 时陀螺仪/加速度计输出均为 1kHz */
    mpu6050_WriteReg(REG_SMPLRT_DIV, 0x00);

    /* 5. DLPF 配置: 42Hz 低通滤波, 1kHz 内部采样
     *    手肘挥动频带 <10Hz, 42Hz 带宽保留足够响应同时滤除电机振动噪声 */
    mpu6050_WriteReg(REG_CONFIG, DLPF_CFG_42HZ);

    /* 6. 陀螺仪满量程: ±500°/s
     *    手肘运动角速度 ≤500°/s, 分辨率 65.5 LSB/(°/s) */
    mpu6050_WriteReg(REG_GYRO_CONFIG, GYRO_FS_SEL_500);

    /* 7. 加速度计满量程: ±4g
     *    手肘运动加速度 2~4g, 分辨率 8192 LSB/g */
    mpu6050_WriteReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL_4G);

    /* 8. 关闭中断 (INT 引脚悬空, 使用轮询模式) */
    mpu6050_WriteReg(REG_INT_ENABLE, 0x00);

    /* 9. 关闭 FIFO, 关闭 I2C 主模式 (直读寄存器) */
    mpu6050_WriteReg(REG_USER_CTRL, 0x00);

    return 0;
}

/*===========================================================================
 * 单位转换
 *===========================================================================*/
float MPU6050_GyroToDPS(int16_t raw)
{
    return (float)raw / GYRO_SCALE_500;      /* LSB → °/s */
}

float MPU6050_AccelToG(int16_t raw)
{
    return (float)raw / ACCEL_SCALE_4G;      /* LSB → g */
}

/*===========================================================================
 * 读取加速度计原始数据 (3轴 × 16bit Big-Endian)
 * 范围: ±4g → ±32768,  8192 LSB/g
 *===========================================================================*/
void MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6] = {0};
    if(mpu6050_ReadMulti(REG_ACCEL_XOUT_H, buf, 6) != 0) return;/* 失败保持上次值 */
    *ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
}

/*===========================================================================
 * 读取陀螺仪原始数据 (3轴 × 16bit Big-Endian)
 * 范围: ±500°/s → ±32768,  65.5 LSB/(°/s)
 *===========================================================================*/
void MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6] = {0};
    if(mpu6050_ReadMulti(REG_GYRO_XOUT_H, buf, 6) != 0) return;/* 失败保持上次值 */
    *gx = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *gy = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *gz = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
}

/*===========================================================================
 * 读取温度原始数据
 * 公式: Temperature(°C) = (TEMP_OUT / 340.0) + 36.53
 *===========================================================================*/
int16_t MPU6050_ReadTemp(void)
{
    uint8_t buf[2] = {0};
    mpu6050_ReadMulti(REG_TEMP_OUT_H, buf, 2);
    return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}


/*===========================================================================
 * 陀螺仪零偏校准
 * 采样期间模块必须保持绝对静止!
 * 采样 400 次 × 2.2ms/次 ≈ 0.9s, 取平均值作为零偏
 *
 * 说明: 加速度计零偏不再采集 —— 倾角直接用原始加速度计算
 * (减零偏会破坏重力基准, 见姿态解算函数内注释)
 *
 * 返回: 0=成功
 *===========================================================================*/
uint8_t MPU6050_Calibrate_GyroAcc(mpu6050_attitude_t *att)
{
    uint16_t i = 0;
    int16_t gx, gy, gz;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const uint16_t samples = 400;  /* 400 次采样 × 2.2ms ≈ 0.9s */

    for (i = 0; i < samples; i++)
    {
        MPU6050_ReadGyro(&gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
    }

    /* 平均陀螺仪零偏 (LSB), 使用时与原始读数相减 */
    att->gx_offset = (int16_t)(gx_sum / samples);
    att->gy_offset = (int16_t)(gy_sum / samples);
    att->gz_offset = (int16_t)(gz_sum / samples);
    return 0;
}


/*===========================================================================
 * DCM 互补滤波姿态解算 (Mahony 矩阵版, 不用四元数)
 *
 * 内部用 3×3 方向余弦矩阵 R (机体→世界) 积分姿态 —— 无欧拉角奇点,
 * 输出时再用 atan2 提取欧拉角, 规避 pitch=±90° 的万向锁。
 *
 * R 约定: ZYX (yaw → pitch → roll), 与加速度倾角公式一致:
 *   pitch = -atan2(ax, sqrt(ay^2+az^2)), roll = atan2(ay, az)
 * 重力方向在机体系中的估计 = R 第三行 [R[2][0], R[2][1], R[2][2]]。
 *===========================================================================*/

// static float R[3][3] = {   /* 旋转矩阵 (机体→世界), 初始单位阵 */
//     {1.0f, 0.0f, 0.0f},
//     {0.0f, 1.0f, 0.0f},
//     {0.0f, 0.0f, 1.0f}
// };

// #define DCM_KP           2.0f      /* 加速度校正比例增益 (1/s), 时间常数约 1/Kp = 0.5s */
// #define ACCEL_NORM_MIN   7373.0f   /* 0.9g × 8192 LSB/g, 加速度模长下限 */
// #define ACCEL_NORM_MAX   9011.0f   /* 1.1g × 8192 LSB/g, 加速度模长上限 */

// /*
//  * 姿态解算 (DCM 互补滤波)
//  * 输入: att 结构体 (原始数据 + 时间戳)
//  * 输出: att->pitch, att->roll, att->yaw 更新 (单位: 度)
//  */
// void MPU6050_AttitudeUpdate(mpu6050_attitude_t *att)
// {
//     uint32_t nowtime = HAL_GetTick();  /* 当前时间(ms) */
//     float dt = (float)(nowtime - att->last_time) * 0.001f;  /* 微分时间(s) */
//     att->last_time = nowtime;

//     /* dt 保护: 大 dt 会使一阶积分失准, 截断到 10ms */
//     if (dt <= 0.0f) return;
//     if (dt > 0.05f) dt = 0.01f;

//     /* 读取传感器原始数据 */
//     MPU6050_ReadGyro(&att->gyro_x, &att->gyro_y, &att->gyro_z);
//     MPU6050_ReadAccel(&att->accel_x, &att->accel_y, &att->accel_z);

//     /* 角速度 (rad/s): 减零偏, LSB→°/s→rad/s */
//     float wx = (float)(att->gyro_x - gx_offset) / GYRO_SCALE_500 * DEG_TO_RAD;
//     float wy = (float)(att->gyro_y - gy_offset) / GYRO_SCALE_500 * DEG_TO_RAD;
//     float wz = (float)(att->gyro_z - gz_offset) / GYRO_SCALE_500 * DEG_TO_RAD;

//     /* 加速度模长门控: 仅当 |a|≈1g (静止/匀速) 时加速度可信;
//      * 线加速/振动时跳过加速度校正, 只用陀螺仪积分, 避免倾角被带偏 */
//     float ax = (float)att->accel_x;
//     float ay = (float)att->accel_y;
//     float az = (float)att->accel_z;
//     float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
//     uint8_t accel_valid = (acc_norm > ACCEL_NORM_MIN) && (acc_norm < ACCEL_NORM_MAX);

//     if (accel_valid)
//     {
//         /* 归一化加速度 → 机体系「上」方向测量值 */
//         float inv = 1.0f / acc_norm;
//         float ax_n = ax * inv;
//         float ay_n = ay * inv;
//         float az_n = az * inv;

//         /* 误差 e = a_meas × g_est (g_est 为 R 第三行), 指向把 g_est 转到 a_meas 的旋转轴 */
//         float ex = (ay_n * R[2][2]) - (az_n * R[2][1]);
//         float ey = (az_n * R[2][0]) - (ax_n * R[2][2]);
//         float ez = (ax_n * R[2][1]) - (ay_n * R[2][0]);

//         /* 比例校正: 误差反馈回角速度 (Mahony), 只校正倾斜、不校正偏航 */
//         wx += DCM_KP * ex;
//         wy += DCM_KP * ey;
//         wz += DCM_KP * ez;
//     }

//     /* 陀螺仪一阶更新: R = R * (I + [w]× * dt), [w]× 为斜对称矩阵 */
//     {
//         float om01 = -wz * dt, om02 =  wy * dt;
//         float om10 =  wz * dt, om12 = -wx * dt;
//         float om20 = -wy * dt, om21 =  wx * dt;

//         float dR00 = R[0][1]*om10 + R[0][2]*om20;
//         float dR01 = R[0][0]*om01 + R[0][2]*om21;
//         float dR02 = R[0][0]*om02 + R[0][1]*om12;
//         float dR10 = R[1][1]*om10 + R[1][2]*om20;
//         float dR11 = R[1][0]*om01 + R[1][2]*om21;
//         float dR12 = R[1][0]*om02 + R[1][1]*om12;
//         float dR20 = R[2][1]*om10 + R[2][2]*om20;
//         float dR21 = R[2][0]*om01 + R[2][2]*om21;
//         float dR22 = R[2][0]*om02 + R[2][1]*om12;

//         R[0][0] += dR00; R[0][1] += dR01; R[0][2] += dR02;
//         R[1][0] += dR10; R[1][1] += dR11; R[1][2] += dR12;
//         R[2][0] += dR20; R[2][1] += dR21; R[2][2] += dR22;
//     }

//     /* 正交化 (Gram-Schmidt): 数值积分缓慢破坏正交性, 逐周期校正 */
//     {
//         float c0x = R[0][0], c0y = R[1][0], c0z = R[2][0];  /* 第一列 (机身 X 轴) */
//         float c1x = R[0][1], c1y = R[1][1], c1z = R[2][1];  /* 第二列 (机身 Y 轴) */

//         float n0 = sqrtf(c0x*c0x + c0y*c0y + c0z*c0z);
//         c0x /= n0; c0y /= n0; c0z /= n0;

//         float dot = c1x*c0x + c1y*c0y + c1z*c0z;
//         c1x -= dot*c0x; c1y -= dot*c0y; c1z -= dot*c0z;
//         float n1 = sqrtf(c1x*c1x + c1y*c1y + c1z*c1z);
//         c1x /= n1; c1y /= n1; c1z /= n1;

//         /* 第三列 = 前两列叉乘, 保证右手系 */
//         float c2x = c0y*c1z - c0z*c1y;
//         float c2y = c0z*c1x - c0x*c1z;
//         float c2z = c0x*c1y - c0y*c1x;

//         R[0][0] = c0x; R[1][0] = c0y; R[2][0] = c0z;
//         R[0][1] = c1x; R[1][1] = c1y; R[2][1] = c1z;
//         R[0][2] = c2x; R[1][2] = c2y; R[2][2] = c2z;
//     }

//     /* 提取欧拉角 (ZYX, 单位: 度) */
//     att->pitch = atan2f(-R[2][0], sqrtf(R[2][1]*R[2][1] + R[2][2]*R[2][2])) * RAD_TO_DEG;
//     att->roll  = -atan2f(R[2][1], R[2][2]) * RAD_TO_DEG;  /* 负号: 使横滚输出方向与手部运动一致 */
//     att->yaw   = atan2f(R[1][0], R[0][0]) * RAD_TO_DEG;
// }


float acce_x = 0.00f, acce_y = 0.00f;
float gyro_x = 0.00f, gyro_y = 0.00f, gyro_z = 0.00f;
// float k_x = 0.00f,    k_y = 0.00f;
float e_P[2][2] = {{1.00, 0.00}
                  ,{0.00, 1.00}};  /* 误差协方差矩阵 */
float k_k[2][2] = {{0.00, 0.00}
                  ,{0.00, 0.00}};  /* 卡尔曼增益矩阵 */
#define KALMAN_Q  0.0025f
#define KALMAN_R  0.3f
void MPU6050_KalmanUpdate(mpu6050_attitude_t *att)
{
    uint32_t nowtime = HAL_GetTick();  /* 当前时间(ms) */
    float dt = (float)(nowtime - att->last_time) / 1000;  /* 微分时间(s) */
    att->last_time = nowtime;
    if(dt <= 0.0f) return;
    if(dt > 0.05f) dt = 0.01; /* 超出限制， */

    MPU6050_ReadAccel(&att->accel_x, &att->accel_y, &att->accel_z);
    MPU6050_ReadGyro( &att->gyro_x, &att->gyro_y, &att->gyro_z);
    //原始加速度数据 转 加速度常用单位
    float ax = (float)att->accel_x / ACCEL_SCALE_4G;
    float ay = (float)att->accel_y / ACCEL_SCALE_4G;
    float az = (float)att->accel_z / ACCEL_SCALE_4G;

    // 计算加速度计的倾角， 测量值 —— atan2f 返回弧度，必须乘 RAD_TO_DEG 转成度，
    // 才能和下面的 gyro_x/gyro_y（度）在同一单位下做卡尔曼融合
    acce_x = atan2f(ay, az) * RAD_TO_DEG;                              /* roll  测量值 (度) */
    acce_y = -atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;       /* pitch 测量值 (度) */
    //原始陀螺仪数据去除零漂 转 角速度常用单位
    float gx = (float)(att->gyro_x - att->gx_offset) / GYRO_SCALE_500;
    float gy = (float)(att->gyro_y - att->gy_offset) / GYRO_SCALE_500;
    float gz = (float)(att->gyro_z - att->gz_offset) / GYRO_SCALE_500;
    // 先验估计，预测值 —— tan/sin/cos 入参是弧度，att->roll/pitch 是度，先转成弧度
    float roll_rad  = att->roll  * DEG_TO_RAD;
    float pitch_rad = att->pitch * DEG_TO_RAD;
    gyro_x = att->roll + dt * (gx + tanf(pitch_rad) * sinf(roll_rad) * gy + tanf(pitch_rad) * cosf(roll_rad) * gz);
    gyro_y = att->pitch + dt * (cosf(roll_rad) * gy - sinf(roll_rad) * gz);
    // gyro_z = att->yaw   + dt * (sin(att->roll) / cos(att->pitch) * gx + cos(att->roll) / cos(att->pitch) * gz);

    e_P[0][0] = e_P[0][0] + KALMAN_Q;
    e_P[1][1] = e_P[1][1] + KALMAN_Q;

    k_k[0][0] = e_P[0][0] / (e_P[0][0] + KALMAN_R);
    k_k[1][1] = e_P[1][1] / (e_P[1][1] + KALMAN_R);

    att->roll = gyro_x + k_k[0][0] * (acce_x - gyro_x);
    att->pitch  = gyro_y + k_k[1][1] * (acce_y - gyro_y);
    // att->yaw   = gyro_z;
    /* 更新误差协方差矩阵*/
    e_P[0][0] = (1 - k_k[0][0]) * e_P[0][0];
    e_P[1][1] = (1 - k_k[1][1]) * e_P[1][1];
}
