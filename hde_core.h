#ifndef HDE_CORE_H
#define HDE_CORE_H

#include <stdint.h>

// Q16.16 fixed point
typedef int32_t q1616_t;

#define Q1616_ONE (1L << 16)

typedef struct {
    q1616_t kp;
    q1616_t ki;
    q1616_t kd;

    q1616_t integral;
    q1616_t prev_err;

    q1616_t out_min;
    q1616_t out_max;
    q1616_t output;
} hde_pid_t; // Total: 8 * int32_t = 32 bytes

void hde_pid_init(hde_pid_t *pid, q1616_t kp, q1616_t ki, q1616_t kd,
                  q1616_t out_min, q1616_t out_max);
q1616_t hde_pid_step(hde_pid_t *pid, q1616_t setpoint, q1616_t meas);

#endif
add hde_core.h
