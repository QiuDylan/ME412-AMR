/* avoid_demo.c - obstacle-avoidance roam (AVOID_DEMO = 1): VL53L0X front + left/right (35 deg, via mux) + Yahboom + MPU-6050.
 * Turns toward the side sensor that reads farther. Without the mux: front sensor only, turns AVD_TURN_DIR.
 * START (BOOT button or Enter in the nc client) = go; press again = stop. 'x' + Enter = wheels off.
 *   FWD  : drive straight; slow down below AVD_SLOW_MM; front < AVD_TURN_MM or side < AVD_SIDE_MM -> avoid
 *   BACK : (only if closer than AVD_BACK_MM) reverse AVD_BACK_MS so the spin doesn't clip it
 *   TURN : spin AVD_TURN_DIR until the front reads > AVD_CLEAR_MM (after >= AVD_MIN_TURN_DEG),
 *          then AVD_EXTRA_DEG more so the robot's width clears the edge -> FWD.
 *          Never turns longer than AVD_TURN_MAX_MS / AVD_MAX_TURN_DEG: then drives forward anyway.
 * Turn angle from the gyro; wheel encoders if the MPU is missing. Sensor lost -> wheels stop. */
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum { IDLE, FWD, BACK, TURN } st_t;
static const char *name[] = { "IDLE", "FWD", "BACK", "TURN" };
static bool imu;

static float heading_deg(void)                 /* + = CCW */
{
    static float last;
    if (imu) return last = mpu_yaw_deg();
    int32_t c[4];
    if (yb_counts(c) == ESP_OK) last = (-c[0] + c[1] - c[2] + c[3]) / 4.0f * 360.0f / IRD_COUNTS_PER_360;
    return last;
}

static bool start_pressed(void)                /* once per press (100 ms release debounce), or Enter */
{
    static int up;
    bool p = !gpio_get_level(START_BTN_PIN), press = p && up >= 5;
    up = p ? 0 : up + 1;
#if NET_LOG
    if (net_take_start()) press = true;
#endif
    return press;
}

static void drive(int v) { yb_wheels((const int16_t[4]){ (int16_t)v, (int16_t)v, (int16_t)v, (int16_t)v }); }

void avoid_demo_main(void)
{
    bool motors = yb_init();                    /* first: stops wheels left running before a reset */
    printf("AVD: last reset: %s\n", reset_why());
#if NET_LOG
    net_init();
#endif
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << START_BTN_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 });
    imu = mpu_init(yb_bus());
    bool tof = tof_start();
    printf("AVD: motors %s, ToF %s, turns by %s\n", motors ? "ok" : "MISSING", tof ? "ok" : "MISSING",
           imu ? "gyro" : "wheel encoders");
    if (!motors || !tof) {
        printf("AVD: not driving - fix the missing part and reset\n");
        for (;;) { vTaskDelay(pdMS_TO_TICKS(500)); if (tof) printf("AVD: front %d mm\n", tof_mm()); }
    }
    printf("AVD: ready (%d ToF sensor%s). START (BOOT or Enter) = go / stop. Front < %d mm or side < %d mm -> avoid.\n",
           tof_count(), tof_count() > 1 ? "s" : "", AVD_TURN_MM, AVD_SIDE_MM);

    st_t s = IDLE;
    int n = 0, k = 0, dir = AVD_TURN_DIR;
    int64_t ts = 0;
    float h0 = 0, clear_at = 0;
    bool cleared = false;
    TickType_t wake = xTaskGetTickCount();
#define GO(x) (s = (x), ts = now_ms())

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
        int d = tof_mm(), rl = tof_get(1), rr = tof_get(2);   /* front, left (+35), right (-35); -1 = none */
        int L = rl < 0 ? TOF_NONE_MM : rl, R = rr < 0 ? TOF_NONE_MM : rr;   /* missing side = ignore */
        float h = heading_deg();
        int64_t t = now_ms() - ts;

        if (start_pressed()) {
            if (s == IDLE) { n = 0; printf("AVD: GO\n"); GO(FWD); }
            else { yb_brake(); printf("AVD: stopped by START (%d avoids)\n", n); GO(IDLE); }
        }
        if (++k % 10 == 0)                                     /* status 5 Hz */
            printf("AVD: %-4s F %4d  L %4d  R %4d mm  heading %4.0f  avoids %d  tof-resets %d\n",
                   name[s], d, rl, rr, (double)h, n, tof_reinits());

        switch (s) {
        case IDLE:
            if (k % 10 == 0) yb_brake();
            break;
        case FWD:
            if (d < 0) {                                       /* no fresh front reading: don't drive blind */
                yb_brake();
                if (k % 25 == 0) printf("AVD: no front ToF reading - holding\n");
            } else if (d < AVD_TURN_MM || L < AVD_SIDE_MM || R < AVD_SIDE_MM) {
                yb_brake(); n++;
                /* turn toward the side with more room; equal (or no side sensors) -> AVD_TURN_DIR */
                dir = L > R + 50 ? 1 : R > L + 50 ? -1 : AVD_TURN_DIR;
                printf("AVD: obstacle F %d L %d R %d mm -> avoid #%d, turning %s\n", d, rl, rr, n, dir > 0 ? "left" : "right");
                if (d < AVD_BACK_MM) GO(BACK);
                else { GO(TURN); h0 = h; cleared = false; }
            } else {
                int v = AVD_FWD_SPEED;
                if (d < AVD_SLOW_MM) v = AVD_MIN_SPEED + (AVD_FWD_SPEED - AVD_MIN_SPEED) * (d - AVD_TURN_MM) / (AVD_SLOW_MM - AVD_TURN_MM);
                drive(v);
            }
            break;
        case BACK:
            if (t < AVD_BACK_MS) drive(-AVD_BACK_SPEED);
            else { yb_brake(); GO(TURN); h0 = h; cleared = false; }
            break;
        case TURN: {
            float turned = fabsf(h - h0);
            bool clear = d > AVD_CLEAR_MM && L > AVD_SIDE_MM && R > AVD_SIDE_MM;
            if (!cleared && turned >= AVD_MIN_TURN_DEG && clear) { cleared = true; clear_at = turned; }
            float extra = tof_count() > 1 ? AVD_EXTRA_DEG_SIDES : AVD_EXTRA_DEG;
            bool timeout = t >= AVD_TURN_MAX_MS;
            if ((cleared && turned >= clear_at + extra) || turned >= AVD_MAX_TURN_DEG || timeout) {
                yb_brake();
                printf("AVD: turned %.0f deg%s, F %d L %d R %d mm -> forward\n", (double)turned,
                       timeout ? " (time limit)" : "", d, rl, rr);
                GO(FWD);
            } else {
                yb_spin(dir * AVD_TURN_SPEED);
            }
            break; }
        }
    }
}
