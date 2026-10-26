/* ir_tracker.c - IR bearing to opponent base / opponent beacon (weighted centroid of array)
 * Owns the shared ADC1 unit (dock sensor in return_home.c also uses it). */
#include "robot.h"

static adc_oneshot_unit_handle_t adc;
static const adc_channel_t ch[N_IR] = IR_ADC_CH;
static const float ang[N_IR] = IR_ANGLES;

void adc_add(adc_channel_t c)
{
    adc_oneshot_config_channel(adc, c, &(adc_oneshot_chan_cfg_t){ .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 });
}
int adc_read(adc_channel_t c) { int v = 0; adc_oneshot_read(adc, c, &v); return v; }

void ir_init(void)
{
    adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){ .unit_id = ADC_UNIT_1 }, &adc);
    for (int i = 0; i < N_IR; i++) adc_add(ch[i]);
}

void ir_read(ir_t *r)
{
    int s = 0, pk = 0; float ws = 0;
    for (int i = 0; i < N_IR; i++) { int v = adc_read(ch[i]); s += v; ws += v * ang[i]; if (v > pk) pk = v; }
    r->sum = s; r->peak = pk; r->bearing = s ? ws / s : 0;
}

bool ir_seen(const ir_t *r)     { return r->peak >= IR_DETECT; }
bool ir_in_range(const ir_t *r) { return r->sum >= IR_RANGE_SUM; }

/* Turn toward beacon and creep in until in range. Beacons on the home side are ignored
 * (align.c) so we never chase our own base. TODO: tell robot beacon (20 pts) from base (10). */
bool track_cmd(const ir_t *r, cmd_t *c)
{
    if (!ir_seen(r) || !align_safe(align_yaw() + r->bearing)) return false;
    c->w = KP_TRACK * r->bearing; c->vx = ir_in_range(r) ? 0 : V_APPROACH; c->vy = 0;
    return true;
}

