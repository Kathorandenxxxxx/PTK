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
#define DEG_PER_STEP            (360.0f / (float)STEP_PER_REV)

/* TIM3: 72MHz/(71+1) = 1MHz 计数时钟，16 位 ARR*/
#define STEPPER_TIM_TICK_HZ     1000000u
#define STEP_FREQ_MIN_HZ        20.0f //16位ARR下限≈15.3Hz，留余量；低于此直接停
#define STEP_FREQ_MAX_HZ        1200.0f // 28BYJ-48 不失步上限，按实测调

/* 避免职责杂糅和数据暴露，数据私有*/
typedef struct
{
    volatile uint8_t     step_index; 
    volatile bool        direction;
    volatile int32_t     current_steps;
    volatile uint16_t    arr;
    volatile uint8_t     running;
    int32_t              limit_min,limit_max;
}Stepper_t;

// 外部接口函数
void    Stepper_Init(Stepper_t *m);
void    Stepper_Stop(void);
void    Stepper_SetLimit(Stepper_t *m, int32_t min_steps, int32_t max_steps);
void    Stepper_SetSpeed(Stepper_t *m, float speed_steps_per_sec);
int32_t Stepper_GetSteps(Stepper_t *m);
void    Stepper_TIM_ElapsedCallback(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif

#endif
