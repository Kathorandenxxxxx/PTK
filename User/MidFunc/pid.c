#include "pid.h"

float PID_Update(PID_t *p, float err, float dt)
{
    if(dt < 0.001f) dt = 0.001f;
    if(dt > 0.05)  dt = 0.05f;
    
    if(err < p->deadband)
    {
        p->integral = 0.0f;
        p->previous_error = err;
        return 0.0f;
    }
    /* 抗饱和：试算含新积分项的输出，饱和就不积分 */
    float out_try = p->Kp * err + p->Ki * (p->integral + err * dt);
    if(out_try < p->output_max && out_try > -p->output_max)
    {
        p->integral += err * dt;
        if(p->integral > p->integral_max) p->integral = p->integral_max;
        if(p->integral < -p->integral_max) p->integral = -p->integral_max;
    }
    float derivative = (err - p->previous_error) / dt;
    p->previous_error = err;
    float out = p->Kp * err + p->Ki * p->integral + p->Kd * derivative;
    if(out > p->output_max) out = p->output_max;
    if(out < -p->output_max) out = -p->output_max;
    return out;
}

void PID_Init(PID_t *p, float kp, float ki, float kd, float i_max, float out_max, float deadband)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
    p->integral_max = i_max;
    p->output_max = out_max;
    p->deadband = deadband;
    PID_Reset(p);
}

void PID_Reset(PID_t *p)
{
    p->integral = 0.0f;
    p->previous_error = 0.0f;
}
