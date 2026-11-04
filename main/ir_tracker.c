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

void ir_read(ir_t *r)
{
    int s = 0, pk = 0, v[N_IR]; float x = 0, y = 0;
    for (int i = 0; i < N_IR; i++) { v[i] = adc_read(ch[i]); s += v[i]; x += v[i] * ca[i]; y += v[i] * sa[i]; if (v[i] > pk) pk = v[i]; }
    r->sum = s; r->peak = pk; r->bearing = atan2f(y, x);
#if IR_DEBUG
    printf("IR %4d %4d %4d %4d %4d %4d | brg %4.0f\n", v[0], v[1], v[2], v[3], v[4], v[5], r->bearing * 57.3f);
#endif
}

/* Non-blocking blink (red) while any sensor > IR_LED_THRESH; only touches the LED on change. */
void ir_indicate(const ir_t *r)
{
    static int64_t t0; static bool on;
    int64_t now = now_ms();
    if (r->peak > IR_LED_THRESH) { if (now - t0 >= IR_BLINK_MS) { t0 = now; on = !on; led_rgb(on ? 64 : 0, 0, 0); } }
    else if (on) { on = false; led_rgb(0, 0, 0); }
}

bool ir_seen(const ir_t *r)     { return r->peak >= IR_DETECT; }
bool ir_in_range(const ir_t *r) { return r->sum >= IR_RANGE_SUM; }

/* Rotate hull toward beacon while strafing straight at it (mecanum), stop when in range.
 * Beacons in the direction of our own base are ignored. TODO: tell robot (20 pts) from base (10). */
bool track_cmd(const ir_t *r, cmd_t *c)
{
    if (!ir_seen(r) || !align_safe(align_yaw() + r->bearing)) return false;
    float v = ir_in_range(r) ? 0 : V_APPROACH;
    c->w = KP_TRACK * r->bearing; c->vx = v * cosf(r->bearing); c->vy = v * sinf(r->bearing);
    return true;
}
