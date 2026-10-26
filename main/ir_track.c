/* ir_track.c - IR beacon bearing (weighted centroid of phototransistor array) + dock sensor */
#include "robot.h"

static adc_oneshot_unit_handle_t adc;
static const adc_channel_t ch[N_IR] = IR_ADC_CH;
static const float ang[N_IR] = IR_ANGLES;

void ir_init(void)
{
    adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){ .unit_id = ADC_UNIT_1 }, &adc);
    adc_oneshot_chan_cfg_t cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    for (int i = 0; i < N_IR; i++) adc_oneshot_config_channel(adc, ch[i], &cfg);
    adc_oneshot_config_channel(adc, DOCK_ADC_CH, &cfg);
}

void ir_read(ir_t *r)
{
    int s = 0, pk = 0, v; float ws = 0;
    for (int i = 0; i < N_IR; i++) {
        adc_oneshot_read(adc, ch[i], &v);
        s += v; ws += v * ang[i]; if (v > pk) pk = v;
    }
    r->sum = s; r->peak = pk; r->bearing = s ? ws / s : 0;
}

/* Rotate to centre the beacon, creep forward until within shooting range. */
bool track_cmd(const ir_t *r, cmd_t *c)
{
    if (r->peak < IR_DETECT) return false;
    c->w = KP_TRACK * r->bearing; c->vx = (r->sum < IR_RANGE_SUM) ? V_APPROACH : 0; c->vy = 0;
    return true;
}

bool ir_aligned(const ir_t *r) { return r->peak >= IR_DETECT && r->sum >= IR_RANGE_SUM && fabsf(r->bearing) < IR_AIM_TOL; }

bool dock_detected(void) { int v; adc_oneshot_read(adc, DOCK_ADC_CH, &v); return v > DOCK_BLACK; }

