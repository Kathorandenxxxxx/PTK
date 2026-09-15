#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__
#include "stdint.h"

void stepper_Task(void *parameter);
/* 只读访问接口，供显示层等外部消费者使用 */
float   Motor_GetTargetSteps  (void);
int32_t Motor_GetCurrentSteps (void);

#endif // __MOTOR_TASK_H__
