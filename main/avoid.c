/* avoid.c - interrupt-timed ultrasonic ring + reflexive avoidance
 * (modernised from Ericson Mar's 1998 subsumption AVOID demo; original in git history)
 * Echoes are timed by GPIO edge ISRs, so no CPU time is spent busy-waiting. */
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const int echo[N_SONAR] = SONAR_ECHO_PINS, avoid_cm[N_SONAR] = AVOID_CM;
static const float ux[N_SONAR] = SONAR_UX, uy[N_SONAR] = SONAR_UY;
static volatile int64_t t_rise[N_SONAR];
static volatile int dist_cm[N_SONAR];

static void IRAM_ATTR echo_isr(void *arg)
{
    int i = (int)(intptr_t)arg, d; int64_t t = esp_timer_get_time();
    if (gpio_get_level(echo[i])) { t_rise[i] = t; return; }
    d = (int)((t - t_rise[i]) / US_PER_CM);
    dist_cm[i] = d > SONAR_MAX_CM ? SONAR_MAX_CM : d;
}

static void sonar_task(void *p)     /* fire all sonars together, ISRs collect echoes */
{
    (void)p;
    for (;;) {
        gpio_set_level(SONAR_TRIG_PIN, 1); esp_rom_delay_us(10); gpio_set_level(SONAR_TRIG_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(SONAR_PERIOD_MS));
    }
}

void avoid_init(void)
{
    gpio_set_direction(SONAR_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_install_isr_service(0);
    for (int i = 0; i < N_SONAR; i++) {
        dist_cm[i] = SONAR_MAX_CM;
        gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << echo[i], .mode = GPIO_MODE_INPUT, .intr_type = GPIO_INTR_ANYEDGE });
        gpio_isr_handler_add(echo[i], echo_isr, (void *)(intptr_t)i);
    }
    xTaskCreatePinnedToCore(sonar_task, "sonar", 2048, NULL, 5, NULL, 0);
}

/* 1) scale forward speed down between SLOW_CM and STOP_CM (negative = back off)
 * 2) repulsive vector from every sonar inside its AVOID_CM envelope
 * Only translation is overridden, so the hull can keep rotating toward its target. */
bool avoid_cmd(cmd_t *c)
{
    float fx = 0, fy = 0; int f = dist_cm[SON_F];
    if (c->vx > 0 && f > SONAR_MIN_CM && f < SLOW_CM) c->vx *= (float)(f - STOP_CM) / (SLOW_CM - STOP_CM);
    for (int i = 0; i < N_SONAR; i++) {
        int d = dist_cm[i];
        if (d > SONAR_MIN_CM && d < avoid_cm[i]) { float k = (float)(avoid_cm[i] - d) / avoid_cm[i]; fx -= ux[i] * k; fy -= uy[i] * k; }
    }
    if (fx == 0 && fy == 0) return false;
    if (fx < 0) fy += (dist_cm[SON_L] > dist_cm[SON_R]) ? SIDESTEP : -SIDESTEP;  /* sidestep to open side */
    c->vx = fx * V_AVOID; c->vy = fy * V_AVOID;
    return true;
}
