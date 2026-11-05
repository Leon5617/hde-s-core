#include <stdio.h>
#include "hde_core.h"

// First-order plant simulation
#define PLANT_GAIN Q1616_ONE
#define TAU (10 * Q1616_ONE)

static q1616_t plant_state = 0;

static q1616_t plant_step(q1616_t u)
{
    int64_t term = (int64_t)(TAU - Q1616_ONE) * plant_state + (int64_t)PLANT_GAIN * u;
    plant_state = (q1616_t)(term >> 16);
    return plant_state;
}

int main(void)
{
    hde_pid_t pid;
    hde_pid_init(&pid,
        (q1616_t)(0.8 * Q1616_ONE),
        (q1616_t)(0.05 * Q1616_ONE),
        (q1616_t)(0.1 * Q1616_ONE),
        0,
        Q1616_ONE
    );

    const q1616_t setpoint = Q1616_ONE;
    plant_state = 0;

    for(int n = 0; n < 40; n++)
    {
        q1616_t u = hde_pid_step(&pid, setpoint, plant_state);
        plant_state = plant_step(u);
        printf("%d, %f\n", n, (double)plant_state / Q1616_ONE);
    }
    return 0;
}
add main.c step response demo
