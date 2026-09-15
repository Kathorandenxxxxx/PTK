#ifndef __STEPPER_H
#define __STEPPER_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "stdbool.h"

// 分别定义每个引脚对应的端口和引脚号
#define MOTOR_IN1_GPIO_PORT      GPIOB
#define MOTOR_IN1_GPIO_PIN       GPIO_PIN_5

#define MOTOR_IN2_GPIO_PORT      GPIOB
#define MOTOR_IN2_GPIO_PIN       GPIO_PIN_6

#define MOTOR_IN3_GPIO_PORT      GPIOB
#define MOTOR_IN3_GPIO_PIN       GPIO_PIN_7

#define MOTOR_IN4_GPIO_PORT      GPIOB
#define MOTOR_IN4_GPIO_PIN       GPIO_PIN_8

/*  电机参数：输出轴转一圈需要4096个脉冲 (8拍模式) */
#define STEP_PER_REV             4096
#define PID_DEADBAND             2.0f // 死区范围 (步)

/* TIM3: 72MHz/(71+1) = 1MHz 计数时钟，16 位 ARR*/
#define STEPPER_TIM_TICK_HZ     1000000u
#define STEP_FREQ_MIN_HZ        20.0f //16位ARR下限≈15.3Hz，留余量；低于此直接停
#define STEP_FREQ_MAX_HZ        832.0f // 28BYJ-48 不失步上限，按实测调

/* 机械行程硬限位 */
#define MOTOR_ANGLE_MIN  (-360.0f)
#define MOTOR_ANGLE_MAX  ( 360.0f)
#define MOTOR_STEP_MIN   ((int32_t)(MOTOR_ANGLE_MIN * (float)STEP_PER_REV / 360.0f))
#define MOTOR_STEP_MAX   ((int32_t)(MOTOR_ANGLE_MAX * (float)STEP_PER_REV / 360.0f))

typedef struct{
    volatile uint8_t    step_index; // 当前步进索引 (0~7)
    volatile bool       direction;  // 当前旋转方向 (0=正转, 1=反转)
    volatile float      current_angle; // 当前角度 (-360~360)
    volatile int16_t    current_steps; // 当前步数 (-4096~4095)
    float               target_steps; // 目标步数 (-4096~4095)

    volatile uint16_t   arr; // 当前arr，用于去重
    volatile uint8_t    running;
}motor_state_t;

typedef struct {
    float Kp;  // 比例增益
    float Ki;  // 积分增益
    float Kd;  // 微分增益
    float dt;  // 时间间隔 (秒)
    float integral_max;    // 积分限幅
    float output_max;      // 输出限幅（步/秒）

    float integral;  // 积分累积值
    float error;  // 当前误差
    float previous_error;  // 上一次误差值
    float output;  // PID输出值
    
    float max_accel; //加速度限幅
    float output_ramp; //斜坡后实际速度
}PID_Handle_t;

// 外部接口函数
uint8_t Stepper_PID_UpdateSpeed(PID_Handle_t *pid, motor_state_t *motor, float target_angle);
void    Stepper_Init(void);
void    Stepper_Stop(void);
void    Stepper_SetPhase(motor_state_t *state);
void    Stepper_SetSpeed(float speed_steps_per_sec,motor_state_t *motor);

#ifdef __cplusplus
}
#endif

#endif
