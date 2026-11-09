/* ir_tracker.c - 360 deg BPV11F ring -> bearing to opponent base / beacon (circular centroid)
 * Owns the shared ADC1 unit (dock sensor in return_home.c also uses it).
 * Also: IR detect indicator - blinks the on-board RGB LED while any sensor sees IR. */
#include <stdio.h>
#include "robot.h"
#include "driver/rmt_tx.h"

static adc_oneshot_unit_handle_t adc;
static const adc_channel_t ch[N_IR] = IR_ADC_CH;
static const float ang[N_IR] = IR_ANGLES;
static float ca[N_IR], sa[N_IR];
static rmt_channel_handle_t led_ch;
static rmt_encoder_handle_t led_enc;

void adc_add(adc_channel_t c)
{
    adc_oneshot_config_channel(adc, c, &(adc_oneshot_chan_cfg_t){ .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 });
}
int adc_read(adc_channel_t c) { int v = 0; adc_oneshot_read(adc, c, &v); return v; }

static void led_rgb(uint8_t r, uint8_t g, uint8_t b)   /* WS2812 expects GRB, non-blocking */
{
    static uint8_t px[3]; px[0] = g; px[1] = r; px[2] = b;
    rmt_transmit(led_ch, led_enc, px, 3, &(rmt_transmit_config_t){ 0 });
}

void ir_init(void)
{
    adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){ .unit_id = ADC_UNIT_1 }, &adc);
    for (int i = 0; i < N_IR; i++) { adc_add(ch[i]); ca[i] = cosf(ang[i]); sa[i] = sinf(ang[i]); }
    /* WS2812 on RMT @10 MHz: bit0 = 0.3us H / 0.9us L, bit1 = 0.9us H / 0.3us L */
    rmt_new_tx_channel(&(rmt_tx_channel_config_t){ .gpio_num = RGB_LED_PIN, .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, .mem_block_symbols = 48, .trans_queue_depth = 2 }, &led_ch);
    rmt_new_bytes_encoder(&(rmt_bytes_encoder_config_t){ .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 }, .flags.msb_first = 1 }, &led_enc);
    rmt_enable(led_ch);
    led_rgb(0, 0, 0);
}

/* Oversample each phototransistor, subtract the ring minimum (ambient/sunlight/dark-offset estimate:
 * the sensors facing away from the beacon only see ambient), then take a circular centroid of what is
 * left. The centroid vector is low-passed so the bearing doesn't jitter; "seen" has hysteresis. */
void ir_read(ir_t *r)
{
    static float fx, fy; static bool seen; static int dbg;
    int v[N_IR], mn = 1 << 30, s = 0, pk = 0; float x = 0, y = 0;
    for (int i = 0; i < N_IR; i++) {
        int a = 0; for (int k = 0; k < IR_OVERSAMPLE; k++) a += adc_read(ch[i]);
        v[i] = a / IR_OVERSAMPLE; if (v[i] < mn) mn = v[i];
    }
    for (int i = 0; i < N_IR; i++) {
        v[i] -= mn; s += v[i]; x += v[i] * ca[i]; y += v[i] * sa[i]; if (v[i] > pk) pk = v[i];
    }
    fx += IR_SMOOTH * (x - fx); fy += IR_SMOOTH * (y - fy);
    seen = seen ? pk >= (int)(IR_DETECT * IR_RELEASE_FRAC) : pk >= IR_DETECT;
    r->sum = s; r->peak = pk; r->seen = seen; r->bearing = atan2f(fy, fx);
#if IR_DEBUG
    if (++dbg >= IR_DEBUG_EVERY) {
        dbg = 0;
        printf("IR %4d %4d %4d %4d %4d %4d | amb %4d | pk %4d | brg %4.0f | %s\n", v[0], v[1], v[2], v[3], v[4], v[5],
               mn, pk, r->bearing * 57.3f, seen ? "SEEN" : "-");
    }
#endif
}

/* Non-blocking blink (red) while the tracker has a beacon seen; only touches the LED on change. */
void ir_indicate(const ir_t *r)
{
    static int64_t t0; static bool on;
    int64_t now = now_ms();
    if (r->seen) { if (now - t0 >= IR_BLINK_MS) { t0 = now; on = !on; led_rgb(on ? 64 : 0, 0, 0); } }
    else if (on) { on = false; led_rgb(0, 0, 0); }
}

bool ir_seen(const ir_t *r)     { return r->seen; }
bool ir_in_range(const ir_t *r) { return r->sum >= IR_RANGE_SUM; }

static float clampf(float v, float lim) { return fminf(fmaxf(v, -lim), lim); }

/* Rotate hull toward beacon while strafing straight at it (mecanum), stop when in range.
 * Beacons in the direction of our own base are ignored. TODO: tell robot (20 pts) from base (10). */
bool track_cmd(const ir_t *r, cmd_t *c)
{
    if (!ir_seen(r) || !align_safe(align_yaw() + r->bearing)) return false;
    float v = ir_in_range(r) ? 0 : V_APPROACH;
    c->w = fabsf(r->bearing) > IR_ALIGN_TOL ? clampf(KP_TRACK * r->bearing, IR_W_MAX) : 0;
    c->vx = v * cosf(r->bearing); c->vy = v * sinf(r->bearing);
    return true;
}

/* Rotate-only tracking (bench test / aiming): turn the whole robot until the beacon is dead ahead.
 * Uses only the IR ring (no IMU/map). If the beacon is lost, spin in place toward the side it was
 * last seen on. Returns true while the beacon is seen. */
bool ir_orient_cmd(const ir_t *r, cmd_t *c)
{
    static int64_t t_seen; static float side = 1;
    int64_t now = now_ms();
    if (ir_seen(r)) {
        t_seen = now; side = r->bearing >= 0 ? 1.0f : -1.0f;
        c->w = fabsf(r->bearing) > IR_ALIGN_TOL ? clampf(KP_TRACK * r->bearing, IR_W_MAX) : 0;
        return true;
    }
    if (now - t_seen > IR_LOST_MS) c->w = side * IR_SEARCH_W;
    return false;
}
