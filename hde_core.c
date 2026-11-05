#include "hde_core.h"

void hde_pid_init(hde_pid_t *pid, q1616_t kp, q1616_t ki, q1616_t kd,
                  q1616_t out_min, q1616_t out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral = 0;
    pid->prev_err = 0;
    pid->output = 0;
}

q1616_t hde_pid_step(hde_pid_t *pid, q1616_t setpoint, q1616_t meas)
{
    q1616_t err = setpoint - meas;

    // proportional
    q1616_t p = ( (int64_t)err * pid->kp ) >> 16;

    // integral + simple clamp anti-windup
    pid->integral += err;
    q1616_t i = ( (int64_t)pid->integral * pid->ki ) >> 16;

    // derivative
    q1616_t diff = err - pid->prev_err;
    q1616_t d = ( (int64_t)diff * pid->kd ) >> 16;
    pid->prev_err = err;

    q1616_t raw = p + i + d;

    // output clamp
    if(raw < pid->out_min)
        raw = pid->out_min;
    if(raw > pid->out_max)
        raw = pid->out_max;

    pid->output = raw;
    return raw;
}
add hde_core.c
