/* ir_demo.c - single-BPV11F IR tracking demo on the Yahboom I2C motor driver (IR_DEMO = 1)
 * One narrow-angle phototransistor can't tell left from right, so the robot finds the source by
 * rotating and noting the heading of the brightest reading. Heading = MPU-6050 gyro (IRD_USE_IMU),
 * else wheel encoders; either way kept in "counts" (IRD_COUNTS_PER_360 per turn).
 *   SEARCH : spin up to 360. When the signal rises IRD_DETECT above ambient, keep turning until it
 *            has dropped past its peak, stop, then yaw-control back onto the peak heading -> LOCKED.
 *            Nothing found in the whole turn -> wait IRD_SEARCH_PERIOD_MS (10 s) and spin again.
 *   LOCKED : sit still pointing at it. Only if the source is gone for IRD_LOST_MS (1 s) -> SEARCH.
 * Ambient = darkest reading seen (last full turn / so far in this turn).
 * IRD_CAL = 1: press START to spin, press again when the robot is back on its start mark;
 * the printed count is IRD_COUNTS_PER_360 (only matters for encoder heading).
 * IRD_SENSE_TEST = 1: no tracking - print IR (raw / ambient / signal) and both headings at 5 Hz;
 * START / Enter = one 360 deg CCW turn by gyro (same turn code as the demo), then reports overshoot. */
#include <stdio.h>
#include <stdlib.h>
#include "robot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CNT(deg) ((int32_t)((deg) * IRD_COUNTS_PER_360 / 360.0f))   /* degrees -> rotation counts */

typedef struct { int peak, low; int32_t at, end; bool found; } scan_t;   /* end = heading when the stop was sent */

static int amb = -1;        /* ambient counts, -1 until the first full scan */
static int32_t last_rot, enc_rot, wc[4];   /* wc = last per-wheel counts FL, FR, RL, RR */
static bool imu;            /* MPU-6050 found and in use */

#define TO_DEG(c) ((double)(c) * 360.0 / IRD_COUNTS_PER_360)

/* Robot rotation in counts, + = CCW. Encoders: mean of each wheel's contribution to an in-place spin
 * (always read, for comparison in the prints). IMU: gyro heading scaled to the same counts. */
static int32_t rot(void)
{
    int32_t c[4];
    if (yb_counts(c) == ESP_OK) { enc_rot = (-c[0] + c[1] - c[2] + c[3]) / 4; for (int i = 0; i < 4; i++) wc[i] = c[i]; }
    last_rot = imu ? (int32_t)lroundf(mpu_yaw_deg() * IRD_COUNTS_PER_360 / 360.0f) : enc_rot;
    return last_rot;                /* encoder I2C error: keeps the last value; sweep() times out if stuck */
}

static int ir_raw(void)
{
    int a = 0; for (int k = 0; k < IRD_OVERSAMPLE; k++) a += adc_read(IRD_ADC_CH);
    return a / IRD_OVERSAMPLE;
}

static bool seen(int v) { return amb >= 0 && v - amb >= IRD_DETECT; }

static void led(int v) { if (IRD_LED) ir_indicate(&(ir_t){ .seen = seen(v) }); }

/* START requested: Enter in the WiFi monitor (NET_LOG) - checked once per call */
static bool net_start(void) { return NET_LOG && net_take_start(); }

static void wait_start(void)        /* BOOT button press + release, or Enter over WiFi */
{
    net_start();                    /* drop an Enter typed before we were waiting */
    while (gpio_get_level(START_BTN_PIN)) { if (net_start()) return; vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(30));
    while (!gpio_get_level(START_BTN_PIN)) vTaskDelay(pdMS_TO_TICKS(10));
}

static float coast_s = IRD_COAST_S;   /* learned: seconds of turning that happen after the stop is sent */

/* Rotate to `target` (counts), slowing near the end, logging the brightest and darkest IR on the way.
 * The robot keeps turning after power is cut (inertia + loop delay), so stop early by
 * rate x coast_s, then learn coast_s from the overshoot actually measured.
 * find = true: also stop once the IR source has been detected and the signal has fallen back past
 * its peak (or IRD_PAST_DEG beyond it), so s.at is the beam centre. Sets s.found. */
static scan_t sweep(int32_t target, int speed, bool find)
{
    bool by_ir = false;
    scan_t s = { .peak = -1, .low = 1 << 30 };
    int32_t r = rot(), dir = target > r ? 1 : -1, rp = r;
    int64_t t_end = now_ms() + IRD_SWEEP_TIMEOUT_MS, tp = now_ms();
    float rate = 0, lead = 0;                        /* counts/s in the turn direction; counts to stop early */
    s.at = r;
    for (;;) {
        lead = coast_s * fmaxf(rate, 0);
        float rem = (float)((target - r) * dir) - lead;
        if (rem <= 0) break;
        if (now_ms() > t_end) { printf("IRD: sweep timed out (stalled wheels / encoder read errors?)\n"); break; }
        int sp = (int)fminf((float)speed, fmaxf((float)IRD_MIN_SPEED, IRD_KP * rem));
        yb_spin((int)dir * sp);
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
        r = rot();
        int64_t t = now_ms();
        if (t > tp) rate += 0.4f * ((float)((r - rp) * dir) * 1000.0f / (float)(t - tp) - rate);   /* smoothed */
        rp = r; tp = t;
        int v = ir_raw(); led(v);
        if (v > s.peak) { s.peak = v; s.at = r; }
        if (v < s.low) s.low = v;
        if (find) {
            int base = amb >= 0 && amb < s.low ? amb : s.low;      /* ambient: known, or darkest so far */
            if (!s.found && v - base >= IRD_DETECT) {
                s.found = true;
                printf("IRD: IR detected at %.0f deg  raw %d  ambient %d (%s)  signal %d\n", TO_DEG(r), v, base,
                       base == amb ? "from last turn" : "darkest so far", v - base);
            }
            if (s.found && ((float)(v - base) < IRD_PAST_PEAK * (float)(s.peak - base)
                            || (r - s.at) * dir > CNT(IRD_PAST_DEG))) { amb = base; by_ir = true; break; }
        }
    }
    s.end = r;
    yb_brake();                     /* PWM 0: worm gears stop at once (speed 0 lets the driver's PID ramp down) */
    vTaskDelay(pdMS_TO_TICKS(300));  /* let the hull settle before measuring / the next reading */
    if (by_ir) return s;                             /* stopped on the IR, not at the target: nothing to learn */
    float over = (float)((rot() - target) * dir);    /* + = went past the target */
    if (rate > CNT(10)) {                            /* only learn from turns that were really moving */
        float k = (lead + over) / rate;              /* coast time that would have landed exactly */
        coast_s += 0.5f * (fminf(fmaxf(k, 0), 3.0f) - coast_s);
    }
    printf("IRD: turn end  overshoot %+.1f deg  (stopped %.1f deg early at %.0f deg/s)  coast now %.2f s\n",
           TO_DEG(over), TO_DEG(lead), TO_DEG(rate), (double)coast_s);
    return s;
}

/* Yaw control: closed-loop correction onto `target` (counts) after a coarse turn. Proportional turn
 * speed (floored at IRD_MIN_SPEED so it actually moves, either direction), hard stop once inside
 * +-IRD_TOL_DEG; done when it has stayed inside for 100 ms. Gives up after IRD_HOLD_MS. */
static void face(int32_t target)
{
    int64_t t0 = now_ms(); int in = 0, nudges = 0; bool was_in = true;
    int32_t e = target - rot();
    while (now_ms() - t0 < IRD_HOLD_MS) {
        e = target - rot();
        if (abs(e) <= CNT(IRD_TOL_DEG)) {
            if (!was_in) yb_brake();
            was_in = true;
            if (++in >= 5) break;
        } else {
            if (was_in) nudges++;
            was_in = false; in = 0;
            int sp = (int)fminf((float)IRD_HOLD_MAX, fmaxf((float)IRD_MIN_SPEED, IRD_HOLD_KP * (float)abs(e)));
            yb_spin(e > 0 ? sp : -sp);
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
    yb_brake();
    vTaskDelay(pdMS_TO_TICKS(150));
    e = target - rot();
    printf("IRD: yaw control  error %+.1f deg after %d nudge(s), %.1f s%s\n", -TO_DEG(e), nudges,
           (now_ms() - t0) / 1000.0, abs(e) > CNT(IRD_TOL_DEG) ? "  (outside tolerance - gave up)" : "");
}

/* One search turn (up to 360). Returns true, pointing at the source, if it was found. */
static bool search(void)
{
    int32_t r0 = rot();
    printf("IRD: SEARCH - spinning to look for the IR source\n");
    scan_t s = sweep(r0 + IRD_COUNTS_PER_360, IRD_SEARCH_SPEED, true);
    if (!s.found) {
        amb = s.low;                                      /* full turn: darkest = ambient */
        printf("IRD: nothing found (peak %d, ambient %d, need +%d) - next spin in %d s\n",
               s.peak, amb, IRD_DETECT, IRD_SEARCH_PERIOD_MS / 1000);
        return false;
    }
    face(s.at);                                           /* yaw control onto the brightest heading */
    int now = ir_raw();
    printf("IRD: LOCKED - peak raw %d at %.0f deg, ambient %d -> peak signal %d; after yaw control raw %d, signal %d\n",
           s.peak, TO_DEG(s.at), amb, s.peak - amb, now, now - amb);
    return true;
}

void ir_demo_main(void)
{
    bool motors = yb_init();                              /* first: stops wheels left running before a reset */
    printf("IRD: last reset: %s\n", reset_why());
#if NET_LOG
    net_init();
#endif
    gpio_config(&(gpio_config_t){ .pin_bit_mask = 1ULL << START_BTN_PIN, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 });
    ir_init();                                            /* ADC1 unit (GPIO5 = ADC1_CH4) + RGB LED */
    if (!motors && !IRD_SENSE_TEST) { for (;;) vTaskDelay(portMAX_DELAY); }
#if IRD_USE_IMU
    imu = mpu_init(yb_bus());
    printf("IRD: heading from %s\n", imu ? "MPU-6050 gyro" : "wheel encoders (no IMU)");
#endif

#if IRD_SENSE_TEST
    printf("SENSE TEST: ambient = darkest reading since boot. Point the IR source at the sensor and\n"
           "            watch sig / SEEN (needs >= %d). START or Enter = one 360 deg CCW turn by gyro%s.\n",
           IRD_DETECT, motors ? "" : " (motors offline)");
    int n = 0, up = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
        int v = ir_raw();
        if (amb < 0 || v < amb) amb = v;
        led(v);
        rot();
        /* button counts once per press: re-armed only after it has read released for 100 ms (debounce) */
        bool pressed = !gpio_get_level(START_BTN_PIN), press = pressed && up >= 5;
        up = pressed ? 0 : up + 1;
        if ((press || net_start()) && motors) {
            if (!imu) { printf("SENSE TEST: no IMU - 360 turn needs the gyro\n"); continue; }
            float y0 = mpu_yaw_deg(); int32_t e0 = enc_rot, r0 = rot(); int64_t t0 = now_ms();
            printf("SENSE TEST: turning 360 deg CCW (gyro)...\n");
            scan_t sc = sweep(r0 + IRD_COUNTS_PER_360, IRD_SEARCH_SPEED, false);   /* same turn code as the demo */
            face(r0 + IRD_COUNTS_PER_360);                                  /* yaw control onto exactly 360 */
            vTaskDelay(pdMS_TO_TICKS(500));                                 /* let it coast to rest */
            rot();
            float turned = mpu_yaw_deg() - y0;
            printf("SENSE TEST: done in %.1f s  imu %.1f deg (overshoot %+.1f; stop sent at %.1f -> %.1f deg after it)  enc %.1f deg\n"
                   "            IR peak %d at %.0f deg, low %d\n",
                   (now_ms() - t0) / 1000.0, (double)turned, (double)(turned - 360.0f), TO_DEG(sc.end - r0),
                   (double)turned - TO_DEG(sc.end - r0), TO_DEG(enc_rot - e0), sc.peak, TO_DEG(sc.at - r0), sc.low);
            up = 0;                                                         /* ignore a press held through the turn */
        }
        if (motors && n == 0) yb_brake();                     /* idle: re-send PWM 0 a few times a second */
        if (++n >= IRD_DEBUG_EVERY) {
            n = 0;
            printf("IR raw %4d  amb %4d  sig %4d %s | imu %7.1f deg | enc %7.1f deg | wheels FL %ld FR %ld RL(M3) %ld RR %ld\n",
                   v, amb, v - amb, seen(v) ? "SEEN" : "    ", imu ? (double)mpu_yaw_deg() : 0.0, TO_DEG(enc_rot),
                   (long)wc[0], (long)wc[1], (long)wc[2], (long)wc[3]);
        }
    }
#endif

#if YB_WHEEL_TEST
    static const char *name[4] = { "FL (front-left)", "FR (front-right)", "RL (rear-left)", "RR (rear-right)" };
    static const int wmap[4] = YB_WHEEL_MAP;
    for (;;) {
        printf("WHEEL TEST: press START. Each wheel should turn so it would drive the robot FORWARD.\n");
        wait_start();
        for (int i = 0; i < 4; i++) {
            int16_t w[4] = { 0 }; w[i] = 150;
            printf("WHEEL TEST: %s  (driver M%d)\n", name[i], wmap[i] + 1);
            for (int t = 0; t < 2000; t += LOOP_MS) { yb_wheels(w); vTaskDelay(pdMS_TO_TICKS(LOOP_MS)); }
            yb_wheels((const int16_t[4]){ 0 }); vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif

#if IRD_CAL
    for (;;) {
        printf("IRD CAL: put a mark under the robot's nose, press START to begin spinning CCW\n");
        wait_start();
        int32_t r0 = rot();
        yb_spin(IRD_MIN_SPEED + 20);
        printf("IRD CAL: press START when the robot is back on the mark\n");
        while (gpio_get_level(START_BTN_PIN) && !net_start()) { yb_spin(IRD_MIN_SPEED + 20); vTaskDelay(pdMS_TO_TICKS(LOOP_MS)); rot(); }
        yb_spin(0);
        vTaskDelay(pdMS_TO_TICKS(300));
        printf("IRD CAL: one turn = %ld counts  -> set IRD_COUNTS_PER_360 in config.h\n", (long)(rot() - r0));
        while (!gpio_get_level(START_BTN_PIN)) vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif

    printf("IRD: IR tracking demo ready (sensor ADC1 ch %d). Press START (BOOT button).\n", IRD_ADC_CH);
    wait_start();

    bool locked = false; int dbg = 0;
    int64_t t_scan = now_ms() - IRD_SEARCH_PERIOD_MS;     /* spin immediately on start */
    int64_t t_seen = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
        int v = ir_raw(); led(v);
        rot();
        if (++dbg >= IRD_DEBUG_EVERY) {
            dbg = 0;
            printf("IRD %-6s raw %4d  amb %4d  sig %4d %s  hdg %6.1f deg | wheels FL %ld FR %ld RL %ld RR %ld\n",
                   locked ? "LOCKED" : "WAIT", v, amb, amb >= 0 ? v - amb : 0, seen(v) ? "SEEN" : "    ",
                   TO_DEG(last_rot), (long)wc[0], (long)wc[1], (long)wc[2], (long)wc[3]);
            yb_brake();                                   /* standing still: re-assert stop on all wheels (5 Hz) */
        }
        /* locked: stays "seen" down to IRD_RELEASE x IRD_DETECT (hysteresis: noise at the edge doesn't drop it) */
        if (amb >= 0 && v - amb >= (locked ? (int)(IRD_RELEASE * IRD_DETECT) : IRD_DETECT)) t_seen = now_ms();
        if (locked) {
            if (now_ms() - t_seen > IRD_LOST_MS) {        /* gone for 1 s: spin again right away */
                printf("IRD: source lost for %d ms - searching\n", IRD_LOST_MS);
                locked = search();
                t_seen = now_ms(); t_scan = now_ms();
            }
        } else if (seen(v)) {
            locked = true;                                /* appeared while waiting: already facing it */
            printf("IRD: LOCKED - source appeared in front (raw %d, amb %d)\n", v, amb);
        } else if (now_ms() - t_scan >= IRD_SEARCH_PERIOD_MS) {
            locked = search();
            t_seen = now_ms(); t_scan = now_ms();         /* 10 s wait counts from the end of the spin */
        }
    }
}
