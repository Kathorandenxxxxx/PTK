#include "ramp.h"

float Ramp_Update(Ramp_t *r, float target, float dt)
{
    if (dt < 0.001f)  dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;
    float delta_max = r->accel_max * dt;
    float delta = target - r->output_ramp;
    if(delta > delta_max)  delta = delta_max;
    if(delta < -delta_max) delta = -delta_max;
    r->output_ramp += delta;
    return r->output_ramp;
}

void Ramp_Reset(Ramp_t *r, float value)
{
    r->output_ramp = value;
}

void Ramp_Init(Ramp_t *r, float accel_max)
{
    r->accel_max = accel_max;
    r->output_ramp = 0.0f;
}
