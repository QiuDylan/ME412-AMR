/* shooter.c - non-blocking flywheel + servo feeder sequence */
#include "robot.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

static int st; static int64_t t0;   /* 0 idle, 1 spin-up, 2 feeding */

static void duty(int chn, uint32_t d) { ledc_set_duty(LEDC_LOW_SPEED_MODE, chn, d); ledc_update_duty(LEDC_LOW_SPEED_MODE, chn); }
static void servo_us(int us) { duty(LEDC_CH_SERVO, (uint32_t)us * 16384 / (1000000 / SERVO_HZ)); }  /* 14-bit */

void shooter_init(void)   /* call after drive_init(): flywheel shares LEDC_TIMER_0 */
{
    ledc_timer_config(&(ledc_timer_config_t){ .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_14_BIT, .freq_hz = SERVO_HZ, .clk_cfg = LEDC_AUTO_CLK });
    ledc_channel_config(&(ledc_channel_config_t){ .gpio_num = FLYWHEEL_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CH_FLY, .timer_sel = LEDC_TIMER_0, .duty = 0 });
    ledc_channel_config(&(ledc_channel_config_t){ .gpio_num = SERVO_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CH_SERVO, .timer_sel = LEDC_TIMER_1, .duty = 0 });
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << BALL_SW_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 });
    servo_us(SERVO_HOLD_US);
}

void shooter_fire(void) { if (!st) { st = 1; t0 = now_ms(); duty(LEDC_CH_FLY, FLYWHEEL_DUTY); } }

bool shooter_busy(void)
{
    int64_t dt = now_ms() - t0;
    if (st == 1 && dt > SPINUP_MS) { servo_us(SERVO_FIRE_US); st = 2; t0 = now_ms(); }
    else if (st == 2 && dt > FEED_MS) { servo_us(SERVO_HOLD_US); duty(LEDC_CH_FLY, 0); st = 0; }
    return st != 0;
}

bool ball_loaded(void) { return !gpio_get_level(BALL_SW_PIN); }

