/* main.c - Mobile Robot: mission state machine + subsumption-style arbitration
 * Layers (low -> high priority): mission goal -> IR tracking / hull aim -> avoid reflex. */
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum { S_WAIT, S_LEAVE, S_HUNT, S_SHOOT, S_RETURN } state_t;

void app_main(void)
{
    drive_init(); shoot_init(); avoid_init(); ir_init(); home_init();
    align_init();   /* gyro calibration: robot must sit still in dock */
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << START_BTN_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 });

    state_t s = S_WAIT; int64_t ts = 0, t_run = 0; ir_t ir; cmd_t c;
    TickType_t wake = xTaskGetTickCount();
#define GO(x) (s = (x), ts = now_ms())

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
        int64_t t = now_ms() - ts; ir_read(&ir); c = (cmd_t){ 0 };

        switch (s) {
        case S_WAIT:   /* in dock: wait for start (robot placed facing away from home) */
            if (!gpio_get_level(START_BTN_PIN)) { align_zero(); GO(S_LEAVE); t_run = ts; }
            break;
        case S_LEAVE:  /* drive straight out of dock */
            c.vx = V_CRUISE;
            if (t > LEAVE_DOCK_MS) GO(S_HUNT);
            break;
        case S_HUNT:   /* ir_tracker: find opponent base/beacon, close to range, hand off to hull aim */
            if (!ball_loaded() || now_ms() - t_run > RUN_TIMEOUT_MS) { GO(S_RETURN); break; }
            if (track_cmd(&ir, &c)) { if (ir_in_range(&ir)) { shoot_start(align_yaw() + ir.bearing); GO(S_SHOOT); } }
            else { c.vx = 0.5f * V_CRUISE; c.w = W_SEARCH; }   /* TODO: smarter search pattern */
            break;
        case S_SHOOT:  /* shoot: hull aim on IMU, fire; retry hunt if aborted with ball still loaded */
            if (!shoot_update(&ir, &c)) GO(ball_loaded() ? S_HUNT : S_RETURN);
            break;
        case S_RETURN: /* return_home: reload after shot */
            if (home_cmd(&ir, &c)) GO(S_WAIT);
            break;
        }
        if (s == S_LEAVE || s == S_HUNT || s == S_RETURN) avoid_cmd(&c);   /* avoid reflex wins */
        drive_set(c.vx, c.vy, c.w);
    }
}
