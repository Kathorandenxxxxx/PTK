#include "stepper.h"
#include "tim.h"
#include "delay.h"   
#include "stm32f1xx_hal.h"
#include "math.h"


// 8拍1步，一步5.625度
// 8拍控制时序表 (A-AB-B-BC-C-CD-D-DA)
static const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0}, // A
    {1, 1, 0, 0}, // AB
    {0, 1, 0, 0}, // B
    {0, 1, 1, 0}, // BC
    {0, 0, 1, 0}, // C
    {0, 0, 1, 1}, // CD
    {0, 0, 0, 1}, // D
    {1, 0, 0, 1}  // DA
};

static uint32_t s_step_bsrr[8];
static Stepper_t *s;

/* 构建8拍控制时序表*/
static void Stepper_BuildPhaseTable(void)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        uint32_t bsrr = 0;
        if (step_sequence[i][0]) bsrr |= (uint32_t)MOTOR_IN1_GPIO_PIN;
        else                     bsrr |= (uint32_t)MOTOR_IN1_GPIO_PIN << 16u;
        if (step_sequence[i][1]) bsrr |= (uint32_t)MOTOR_IN2_GPIO_PIN;
        else                     bsrr |= (uint32_t)MOTOR_IN2_GPIO_PIN << 16u;
        if (step_sequence[i][2]) bsrr |= (uint32_t)MOTOR_IN3_GPIO_PIN;
        else                     bsrr |= (uint32_t)MOTOR_IN3_GPIO_PIN << 16u;
        if (step_sequence[i][3]) bsrr |= (uint32_t)MOTOR_IN4_GPIO_PIN;
        else                     bsrr |= (uint32_t)MOTOR_IN4_GPIO_PIN << 16u;
        s_step_bsrr[i] = bsrr;
    }
}


/* 换相：仅 ISR 调用。禁止浮点、禁止 RTOS API */
static void Stepper_SetPhase(Stepper_t *m) {
    if(m->direction == 0) { // 正转
        if(m->current_steps >= m->limit_max) return; //硬限位不换相
        MOTOR_IN1_GPIO_PORT->BSRR = s_step_bsrr[m->step_index];
        m->step_index = (m->step_index + 1u) & 7u;
        m->current_steps++;
    } 
    else { // 反转
        if(m->current_steps <= m->limit_min) return; //硬限位不换相
        MOTOR_IN1_GPIO_PORT->BSRR = s_step_bsrr[m->step_index];
        m->step_index = (m->step_index + 7u) & 7u; // 等价于 -1 mod 8
        m->current_steps--;
    }
}

static uint16_t Stepper_FreqToArr(float freq_hz)
{
    if (freq_hz < STEP_FREQ_MIN_HZ) freq_hz = STEP_FREQ_MIN_HZ;
    if (freq_hz > STEP_FREQ_MAX_HZ) freq_hz = STEP_FREQ_MAX_HZ;

    uint32_t arr = (uint32_t)(STEPPER_TIM_TICK_HZ / freq_hz) - 1u;
    if(arr == 0u) arr = 1u;
    if(arr > 0xFFFFu) arr = 0xFFFFu; //定时器计数上限
    return (uint16_t)arr;
}

// 停止电机（所有引脚置低）
void Stepper_Stop(void) {
    MOTOR_IN1_GPIO_PORT->BSRR =
          ((uint32_t)MOTOR_IN1_GPIO_PIN << 16u)
        | ((uint32_t)MOTOR_IN2_GPIO_PIN << 16u)
        | ((uint32_t)MOTOR_IN3_GPIO_PIN << 16u)
        | ((uint32_t)MOTOR_IN4_GPIO_PIN << 16u);
}

/* 原子改写 ARR + CNT 钳位（TIM3 优先级 4，RTOS 临界区屏蔽不住，故用 PRIMASK） */
static void Stepper_WritePeriod(uint16_t arr)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                                   // 用 PRIMASK 而不是 RTOS 临界区，
    htim3.Instance->ARR = (uint32_t)arr;               // 因为 TIM3 优先级 4 屏蔽不住
    if (htim3.Instance->CNT > (uint32_t)arr) htim3.Instance->CNT = 0u;                      // 否则会一路数到 0xFFFF 才翻转(65ms)
    __set_PRIMASK(primask);
}

static void Stepper_TimerEnable(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    __HAL_TIM_SET_COUNTER(&htim3, 0);      // 首拍是完整周期
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
} 

static void Stepper_TimerDisable(void)
{
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim3);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
}

/*===========================================================================
 * 对外接口实现
 *=========================================================================*/
// 初始化（停止电机）
void Stepper_Init(Stepper_t *m) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    Stepper_BuildPhaseTable();
    Stepper_Stop();

    m->step_index = 0;
    m->direction = 0;
    m->current_steps = 0;
    m->arr = Stepper_FreqToArr(STEP_FREQ_MIN_HZ);
    m->running    = 0u;
    m->limit_min  = (int32_t)0x80000000;     /* 默认不限位，由应用层注入 */
    m->limit_max  = (int32_t)0x7FFFFFFF;
    s = m;          /* 绑定为 ISR 活动实例 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, m->arr);
    HAL_TIM_Base_Start(&htim3);   // 计数器常开，避免反复 Start/Stop 的首拍抖动
    Stepper_TimerDisable();
}

void Stepper_SetLimit(Stepper_t *m, int32_t min_steps, int32_t max_steps)
{
    m->limit_min = min_steps;
    m->limit_max = max_steps;
}

void Stepper_SetSpeed(Stepper_t *m, float speed_steps_per_sec) {
    float abs_speed = fabsf(speed_steps_per_sec); // 计算速度
    if(abs_speed < STEP_FREQ_MIN_HZ)
    {
        Stepper_Stop();
        Stepper_TimerDisable();
        m->running = 0u;
        return;
    }

    m->direction = (speed_steps_per_sec > 0.0f) ? 0 : 1; // 设置方向
    
    uint16_t arr = Stepper_FreqToArr(abs_speed);
    if(arr != m->arr)
    {
        Stepper_WritePeriod(arr);
        m->arr = arr;
    }
    if (!m->running) { Stepper_TimerEnable(); m->running = 1; }
}


int32_t Stepper_GetSteps(Stepper_t *m) 
{ 
    return m->current_steps; 
}


void Stepper_TIM_ElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance != TIM3)  return;
    if(s == NULL)               return;
    Stepper_SetPhase(s);
}

