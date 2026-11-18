/* main.c - Mobile Robot: mission state machine + subsumption-style arbitration
 * Layers (low -> high priority): map navigation -> IR tracking / hull aim -> ToF avoid reflex. */
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
typedef enum { S_WAIT, S_LEAVE, S_HUNT, S_SHOOT, S_RETURN, S_TRACK } state_t;
 
#if AVOID_DEMO
static void avd_task(void *arg) { avoid_demo_main(); }
#endif
#if TOF_TEST
static void tof_task(void *arg) { tof_test_main(); }
#endif
#if IR_DEMO
static void demo_task(void *arg) { ir_demo_main(); }
#endif

void app_main(void)
{
#if AVOID_DEMO
    xTaskCreate(avd_task, "avoid", 8192, NULL, 5, NULL);     /* obstacle-avoidance demo */
    vTaskDelete(NULL);
#endif
#if TOF_TEST
    xTaskCreate(tof_task, "tof_test", 8192, NULL, 5, NULL);   /* ToF bench test replaces the IR demo */
    vTaskDelete(NULL);
#endif
#if IR_DEMO
    /* bench demo: one BPV11F + Yahboom driver only; nothing below is initialised. Own task with a big
     * stack - the default 3.5 KB main stack is tight once printf also formats + sends over WiFi. */
    xTaskCreate(demo_task, "ir_demo", 8192, NULL, 5, NULL);
    vTaskDelete(NULL);
#endif
    i2c_master_bus_handle_t bus;
    i2c_new_master_bus(&(i2c_master_bus_config_t){ .i2c_port = I2C_NUM_0, .sda_io_num = I2C_SDA_PIN, .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = 1 }, &bus);
    drive_init(); shoot_init(); ir_init(); home_init(); align_init(bus); avoid_init(bus);
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << START_BTN_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 });
 
    state_t s = S_WAIT; int64_t ts = 0, t_run = 0; ir_t ir; cmd_t c = { 0 };
    TickType_t wake = xTaskGetTickCount();
#define GO(x) (s = (x), ts = now_ms())
 
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
        map_update(&c);                                   /* last command -> dead-reckon, fuse ToF */
        int64_t t = now_ms() - ts; ir_read(&ir); ir_indicate(&ir); c = (cmd_t){ 0 };
 
        switch (s) {
        case S_WAIT:   /* in dock, facing opponent: wait for start */
            if (!gpio_get_level(START_BTN_PIN)) { align_zero(); map_reset(); GO(TRACK_TEST ? S_TRACK : S_LEAVE); t_run = ts; }
            break;
        case S_TRACK:  /* bench test: sense IR, rotate the whole robot to face it (no translation) */
            ir_orient_cmd(&ir, &c);
            break;
        case S_LEAVE:  /* drive straight out of dock */
            c.vx = V_CRUISE;
            if (t > LEAVE_DOCK_MS) GO(S_HUNT);
            break;
        case S_HUNT:   /* map toward opponent side; 360 deg IR ring takes over once a safe beacon is seen */
            if (now_ms() - t_run > RUN_TIMEOUT_MS) { GO(S_RETURN); break; }
            if (track_cmd(&ir, &c)) { if (ir_in_range(&ir)) { shoot_start(align_yaw() + ir.bearing); GO(S_SHOOT); } }
            else map_goto(OPP_X_MM, 0, &c);
            break;
        case S_SHOOT: { /* hull aim on IMU, open CO2 valve; aborted -> keep hunting */
            int r = shoot_update(&ir, &c);
            if (r <= 0) GO(r ? S_HUNT : S_RETURN);
            break; }
        case S_RETURN: /* reload after shot */
            if (home_cmd(&ir, &c)) GO(S_WAIT);
            break;
        }
        if (s == S_LEAVE || s == S_HUNT || s == S_RETURN) avoid_cmd(&c);   /* avoid reflex wins */
        drive_set(c.vx, c.vy, c.w);
    }
}
