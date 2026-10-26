/* main.c - top-level mission state machine + subsumption-style arbitration
 * Layers (low -> high priority): mission/state goal -> IR tracking -> obstacle reflex. */
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum { S_WAIT, S_LEAVE, S_HUNT, S_SHOOT, S_RETURN } state_t;

static float wrap(float a) { return atan2f(sinf(a), cosf(a)); }

void app_main(void)
{
    drive_init(); shooter_init(); obstacle_init(); ir_init();
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << START_BTN_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 });

    state_t s = S_WAIT; int64_t ts = 0; float yaw = 0; ir_t ir; cmd_t c;
    TickType_t wake = xTaskGetTickCount();
#define GO(x) (s = (x), ts = now_ms())

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
        int64_t t = now_ms() - ts; ir_read(&ir); c = (cmd_t){ 0 };

        switch (s) {
        case S_WAIT:   /* placed in dock: wait for start button */
            if (!gpio_get_level(START_BTN_PIN)) { yaw = 0; GO(S_LEAVE); }
            break;
        case S_LEAVE:  /* drive straight out of dock (yaw 0 = facing away from home) */
            c.vx = V_CRUISE;
            if (t > LEAVE_DOCK_MS) GO(S_HUNT);
            break;
        case S_HUNT:   /* only accept IR in the forward half-plane so we never target our own base */
            if (!ball_loaded() || t > RUN_TIMEOUT_MS) { GO(S_RETURN); break; }
            if (cosf(yaw) > 0 && track_cmd(&ir, &c)) { if (ir_aligned(&ir)) { shooter_fire(); GO(S_SHOOT); } }
            else { c.vx = 0.5f * V_CRUISE; c.w = W_SEARCH; }   /* TODO: smarter search pattern */
            break;
        case S_SHOOT:  /* hold still until ball has left */
            if (!shooter_busy()) GO(S_RETURN);
            break;
        case S_RETURN: /* head back (yaw = pi), lock on to own base IR when facing home */
            if (dock_detected()) { GO(S_WAIT); break; }
            if (!(cosf(yaw) < -0.7f && track_cmd(&ir, &c))) c.w = KP_YAW * wrap((float)M_PI - yaw);
            c.vx = V_CRUISE;
            break;
        }
        if (s == S_LEAVE || s == S_HUNT || s == S_RETURN) avoid_cmd(&c);   /* reflex layer wins */
        drive_set(c.vx, c.vy, c.w);
        yaw += c.w * YAW_RATE_MAX * LOOP_MS / 1000.0f;   /* crude dead-reckoning; TODO: IMU */
    }
}

