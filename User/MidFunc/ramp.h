#ifndef __RAMP_H__
#define __RAMP_H__

/* */
typedef struct
{
    /* data */
    float accel_max; //加速度限幅
    float output_ramp; //斜坡后实际速度
}Ramp_t;

void    Ramp_Init(Ramp_t *r, float accel_max);
float    Ramp_Update(Ramp_t *r, float target, float dt);
void   Ramp_Reset(Ramp_t *r, float value);
#endif

