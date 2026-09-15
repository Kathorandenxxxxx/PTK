#ifndef __PID_H_
#define __PID_H_

typedef struct
{
    float Kp;  // 比例增益
    float Ki;  // 积分增益
    float Kd;  // 微分增益
    float integral_max;    // 积分限幅
    float output_max;      // 输出限幅（步/秒）
    float integral;  // 积分累积值
    float previous_error;  // 上一次误差值
    float deadband;
}PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, 
              float i_max, float out_max, float deadband);
void  PID_Reset(PID_t *p);
float PID_Update(PID_t *p, float err, float dt);

#endif
