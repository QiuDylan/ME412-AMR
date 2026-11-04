/* shoot.c - hull aim (PD on IMU heading) + servo-actuated CO2 valve
 * The whole robot rotates to point the fixed barrel at the target heading. IR picks the
 * heading, the IMU holds it: fast, smooth, and doesn't need the beacon in view. */
#include "robot.h"
#include "driver/ledc.h"

static int st; static int64_t t0; static float target;   /* st: 0 idle, 1 aiming, 2 valve open */

static void servo_us(int us)   /* 14-bit duty @ SERVO_HZ */
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_VALVE, (uint32_t)us * 16384 / (1000000 / SERVO_HZ));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_VALVE);
}

void shoot_init(void)
{
    ledc_timer_config(&(ledc_timer_config_t){ .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_14_BIT, .freq_hz = SERVO_HZ, .clk_cfg = LEDC_AUTO_CLK });
    ledc_channel_config(&(ledc_channel_config_t){ .gpio_num = VALVE_SERVO_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CH_VALVE, .timer_sel = LEDC_TIMER_1, .duty = 0 });
    servo_us(VALVE_CLOSED_US);
}

void shoot_start(float heading) { if (!st) { target = wrap_pi(heading); st = 1; t0 = now_ms(); } }

int shoot_update(const ir_t *ir, cmd_t *c)
{
    if (!st) return -1;
    int64_t now = now_ms(); float yaw = align_yaw(), rate = align_rate();
    if (st == 1 && ir_seen(ir)) target = wrap_pi(target + AIM_IR_BLEND * wrap_pi(yaw + ir->bearing - target));
    float err = wrap_pi(target - yaw);
    c->vx = c->vy = 0; c->w = KP_AIM * err - KD_AIM * rate;   /* hull aim */
    if (st == 1) {
        if (!align_safe(target) || now - t0 > AIM_TIMEOUT_MS) { st = 0; return -1; }   /* own base / can't settle */
        if (fabsf(err) < AIM_TOL_RAD && fabsf(rate) < AIM_RATE_TOL) { servo_us(VALVE_OPEN_US); st = 2; t0 = now; }
    } else if (now - t0 > VALVE_OPEN_MS) { servo_us(VALVE_CLOSED_US); st = 0; return 0; }   /* fired */
    return 1;
}
