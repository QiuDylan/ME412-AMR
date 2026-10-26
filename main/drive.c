/* drive.c - mecanum kinematics -> 2x L298N (EN = PWM speed, IN1/IN2 = direction) */
#include "robot.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

static const int en[4] = MOTOR_EN_PINS, in1[4] = MOTOR_IN1_PINS, in2[4] = MOTOR_IN2_PINS, inv[4] = MOTOR_INVERT;

void drive_init(void)
{
    ledc_timer_config(&(ledc_timer_config_t){ .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = MOTOR_PWM_BITS, .freq_hz = MOTOR_PWM_HZ, .clk_cfg = LEDC_AUTO_CLK });
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&(ledc_channel_config_t){ .gpio_num = en[i], .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i, .timer_sel = LEDC_TIMER_0, .duty = 0 });
        gpio_set_direction(in1[i], GPIO_MODE_OUTPUT); gpio_set_direction(in2[i], GPIO_MODE_OUTPUT);
    }
}

/* vx fwd, vy left, w CCW. Wheel order FL, FR, RL, RR. Normalized so no wheel exceeds 1. */
void drive_set(float vx, float vy, float w)
{
    float m[4] = { vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w }, mx = 1.0f;
    for (int i = 0; i < 4; i++) if (fabsf(m[i]) > mx) mx = fabsf(m[i]);
    for (int i = 0; i < 4; i++) {
        int fwd = (m[i] >= 0) ^ inv[i];
        gpio_set_level(in1[i], fwd); gpio_set_level(in2[i], !fwd);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, (uint32_t)(fabsf(m[i]) / mx * MOTOR_PWM_MAX));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}
