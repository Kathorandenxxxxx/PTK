#include "stepper.h"
#include "delay.h"   
#include "stm32f1xx_hal.h"
#include "math.h"
#include "stdlib.h"
// 8拍1步，一步5.625度
// 8拍控制时序表 (A-AB-B-BC-C-CD-D-DA)
const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0}, // A
    {1, 1, 0, 0}, // AB
    {0, 1, 0, 0}, // B
    {0, 1, 1, 0}, // BC
    {0, 0, 1, 0}, // C
    {0, 0, 1, 1}, // CD
    {0, 0, 0, 1}, // D
    {1, 0, 0, 1}  // DA
};

void Stepper_SetPhase(motor_state_t *state) {
    // 根据节拍表分别设置每个引脚状态
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_GPIO_PIN, 
                      step_sequence[state->step_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_GPIO_PIN, 
                      step_sequence[state->step_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_PORT, MOTOR_IN3_GPIO_PIN, 
                      step_sequence[state->step_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_PORT, MOTOR_IN4_GPIO_PIN, 
                      step_sequence[state->step_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if(state->direction == 0) { // 正转
        state->step_index = (state->step_index + 1) % 8;
        state->current_steps++;
    } 
    else { // 反转
        state->step_index = (state->step_index + 7) % 8; // 等价于 -1 mod 8
        state->current_steps--;
    }
}

// 停止电机（所有引脚置低）
void Stepper_Stop(void) {
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_PORT, MOTOR_IN3_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_PORT, MOTOR_IN4_GPIO_PIN, GPIO_PIN_RESET);
}

// 初始化（停止电机）
void Stepper_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    Stepper_Stop();
}
extern TIM_HandleTypeDef htim3; // 定时器句柄，确保在其他文件中定义
uint8_t Stepper_PID_UpdateSpeed(PID_Handle_t *pid, motor_state_t *motor, float target_angle)
{
    motor->target_steps = target_angle * STEP_PER_REV / 360.0f; // 将目标角度转换为步数
    pid->error = motor->target_steps - motor->current_steps; // 计算误差

    // 处理死区
    if(fabsf(pid->error) < PID_DEADBAND) {
        pid->integral = 0; // 在死区内，重置积分
        pid->output = 0;   // 输出为零
        return 1;          // 不需要移动
    }
        
    pid->integral += pid->error * pid->dt; // 积分累积

    // 积分限幅
    if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if(pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    // 微分项
    float derivative = (pid->error - pid->previous_error) / pid->dt; 
    //更新输出
    pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * derivative; // PID计算输出
    // 输出限幅
    if(pid->output > pid->output_max) pid->output = pid->output_max;
    if(pid->output < -pid->output_max) pid->output = -pid->output_max;

    // 更新历史误差
    pid->previous_error = pid->error;
    return 0;
}

void Stepper_SetSpeed(float speed_steps_per_sec, motor_state_t *motor) {
    if(speed_steps_per_sec == 0) {
        Stepper_Stop();
        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE); // 禁用定时器中断
        return;
    }
    motor->direction = (speed_steps_per_sec > 0) ? 0 : 1; // 设置方向
    uint32_t freq_hz = (uint32_t)(fabsf(speed_steps_per_sec)); // 计算速度
    uint32_t arr = 1000000 / freq_hz - 1; // 计算ARR值
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr); // 设置定时器ARR
    if(!(TIM3->DIER & TIM_DIER_UIE))
    {
        __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE); // 启用定时器中断
        HAL_TIM_Base_Start_IT(&htim3); // 启动定时器
    }
}
