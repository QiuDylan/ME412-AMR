/* shoot.c - hull aim (PD on IMU heading) + non-blocking flywheel/servo shot sequence
 * The whole robot rotates to point the fixed shooter at the target heading. IR picks the
 * heading, the IMU holds it: it's fast and smooth, and doesn't need the beacon in view. */
#include "robot.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

static int st; static int64_t t0, t_spin; static float target;   /* st: 0 idle, 1 aiming, 2 feeding */

static void duty(int chn, uint32_t d) { ledc_set_duty(LEDC_LOW_SPEED_MODE, chn, d); ledc_update_duty(LEDC_LOW_SPEED_MODE, chn); }
static void servo_us(int us) { duty(LEDC_CH_SERVO, (uint32_t)us * 16384 / (1000000 / SERVO_HZ)); }  /* 14-bit */
static void stop(void) { servo_us(SERVO_HOLD_US); duty(LEDC_CH_FLY, 0); st = 0; }

void shoot_init(void)   /* call after drive_init(): flywheel shares LEDC_TIMER_0 */
{
    ledc_timer_config(&(ledc_timer_config_t){ .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_14_BIT, .freq_hz = SERVO_HZ, .clk_cfg = LEDC_AUTO_CLK });
    ledc_channel_config(&(ledc_channel_config_t){ .gpio_num = FLYWHEEL_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CH_FLY, .timer_sel = LEDC_TIMER_0, .duty = 0 });
    ledc_channel_config(&(ledc_channel_config_t){ .gpio_num = SERVO_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CH_SERVO, .timer_sel = LEDC_TIMER_1, .duty = 0 });
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << BALL_SW_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 });
    stop();
}

void shoot_start(float heading)
{
    if (st) return;
    target = wrap_pi(heading); st = 1; t0 = t_spin = now_ms();
    duty(LEDC_CH_FLY, FLYWHEEL_DUTY);   /* spin up in parallel with aiming */
}

bool shoot_update(const ir_t *ir, cmd_t *c)
{
    if (!st) return false;
    int64_t now = now_ms(); float yaw = align_yaw(), rate = align_rate();
    if (st == 1 && ir_seen(ir)) target = wrap_pi(target + AIM_IR_BLEND * wrap_pi(yaw + ir->bearing - target));
    float err = wrap_pi(target - yaw);
    c->vx = c->vy = 0; c->w = KP_AIM * err - KD_AIM * rate;   /* hull aim */
    if (st == 1) {
        if (!align_safe(target) || now - t0 > AIM_TIMEOUT_MS) stop();   /* abort: own base / can't settle */
        else if (fabsf(err) < AIM_TOL_RAD && fabsf(rate) < AIM_RATE_TOL && now - t_spin > SPINUP_MS) {
            servo_us(SERVO_FIRE_US); st = 2; t0 = now;
        }
    } else if (now - t0 > FEED_MS) stop();
    return st != 0;
}

bool ball_loaded(void) { return !gpio_get_level(BALL_SW_PIN); }

